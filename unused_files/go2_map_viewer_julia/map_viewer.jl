using HTTP
using JSON3
using GLMakie

const ROSBRIDGE_URL = "ws://localhost:9090"
const MAP_FRAME = "map"
const BASE_FRAME = "base_link"

# Pose of a child frame in its parent. Quaternion is (x, y, z, w).
struct Pose
    x::Float64
    y::Float64
    z::Float64
    qx::Float64
    qy::Float64
    qz::Float64
    qw::Float64
end

const POSE_IDENTITY = Pose(0, 0, 0, 0, 0, 0, 1)

# Latest snapshot the websocket task hands to the GLMakie thread.
# Observables are only written on the main thread.
mutable struct Shared
    lock::ReentrantLock
    status::String
    grid::Matrix{Float64}
    robot::Vector{Point2f}
    generation::Int
    ws::Any  # current HTTP.WebSockets connection, so main() can force it
             # closed on shutdown instead of leaving connection_loop blocked
             # on a receive that may never arrive.
end

# How old the oldest edge in the map->base_link chain can be before the
# status label calls it out as stale (e.g. the localization node died but
# odom->base_link keeps updating, so the chain still resolves structurally).
const TF_STALE_SEC = 5.0

function qmul(ax, ay, az, aw, bx, by, bz, bw)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
end

function rotate(qx, qy, qz, qw, vx, vy, vz)
    tx = 2 * (qy * vz - qz * vy)
    ty = 2 * (qz * vx - qx * vz)
    tz = 2 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )
end

# parent is the pose of frame B in A; child is the pose of frame C in B.
# The result is the pose of C in A.
function compose(parent::Pose, child::Pose)
    rx, ry, rz = rotate(
        parent.qx, parent.qy, parent.qz, parent.qw, child.x, child.y, child.z)
    qx, qy, qz, qw = qmul(
        parent.qx, parent.qy, parent.qz, parent.qw,
        child.qx, child.qy, child.qz, child.qw)
    return Pose(parent.x + rx, parent.y + ry, parent.z + rz, qx, qy, qz, qw)
end

function frame_name(value)
    name = String(value)
    return startswith(name, "/") ? name[2:end] : name
end

function as_float(value)
    return Float64(value)
end

function as_int(value)
    return trunc(Int, as_float(value))
end

function jget(obj, key, default)
    return obj isa JSON3.Object && haskey(obj, key) ? obj[key] : default
end

function publish!(shared::Shared; status=nothing, grid=nothing, robot=nothing)
    lock(shared.lock) do
        status === nothing || (shared.status = status)
        grid === nothing || (shared.grid = grid)
        robot === nothing || (shared.robot = robot)
        shared.generation += 1
    end
end

function parse_pose(item)
    parent = frame_name(item.header.frame_id)
    child = frame_name(item.child_frame_id)
    if isempty(parent) || isempty(child) || parent == child
        return nothing
    end
    tr = item.transform.translation
    rot = item.transform.rotation
    return child, parent, Pose(
        as_float(tr.x), as_float(tr.y), as_float(tr.z),
        as_float(rot.x), as_float(rot.y), as_float(rot.z), as_float(rot.w))
end

# edges: child frame => (parent frame, pose of the child in the parent).
function chain_pose(chain)
    # chain[1] is the source in its parent, chain[2] that parent in its parent, ...
    pose = POSE_IDENTITY
    for edge in chain
        pose = compose(edge, pose)
    end
    return pose
end

#  Returns (pose, age_seconds_of_oldest_edge_in_the_chain) or nothing.
function lookup_pose(edges, target, source)
    chain = Pose[]
    oldest_ts = Inf
    frame = source
    seen = Set{String}()
    for _ in 1:24
        if frame == target
            age = oldest_ts == Inf ? 0.0 : time() - oldest_ts
            return chain_pose(chain), age
        end
        if frame in seen || !haskey(edges, frame)
            return nothing
        end
        push!(seen, frame)
        parent, pose, ts = edges[frame]
        oldest_ts = min(oldest_ts, ts)
        push!(chain, pose)
        frame = parent
    end
    return nothing
end

function apply_tf!(edges, msg)
    transforms = jget(msg, "transforms", nothing)
    transforms isa AbstractVector || return false
    changed = false
    for item in transforms
        parsed = try
            parse_pose(item)
        catch
            nothing
        end
        parsed === nothing && continue
        child, parent, pose = parsed
        edges[child] = (parent, pose, time())
        changed = true
    end
    return changed
end

function occupancy_grid(msg)
    info = msg.info
    width = as_int(info.width)
    height = as_int(info.height)
    resolution = as_float(info.resolution)
    width > 0 && height > 0 && resolution > 0 || return nothing
    cells = msg.data
    cells isa AbstractVector || return nothing
    ncells = widemul(width, height)
    ncells <= length(cells) || return nothing

    # grid[x, y] matches Makie: the first index is x, the second is y,
    # and y increases upward, same as the occupancy grid.
    grid = Array{Float64}(undef, width, height)
    for y in 0:(height - 1), x in 0:(width - 1)
        v = as_int(cells[y * width + x + 1])
        # -1 (unknown) -> mid gray, 0 (free) -> white, 100 (occupied) -> black.
        if v < 0
            grid[x + 1, y + 1] = 0.5
        else
            grid[x + 1, y + 1] = 1.0 - clamp(v, 0, 100) / 100.0
        end
    end

    return (
        grid = grid,
        resolution = resolution,
        origin_x = as_float(info.origin.position.x),
        origin_y = as_float(info.origin.position.y),
    )
end

# Makie places heatmap cell centers on 1:width and 1:height. Cell 0's
# center is half a cell in from the origin, which is Makie coordinate 1.
function robot_point(map_info, pose::Pose)
    gx = (pose.x - map_info.origin_x) / map_info.resolution + 0.5
    gy = (pose.y - map_info.origin_y) / map_info.resolution + 0.5
    return Point2f(gx, gy)
end

function subscribe(ws, topic, type; qos = nothing, throttle_rate = nothing)
    msg = Dict{String, Any}(
        "op" => "subscribe",
        "topic" => topic,
        "type" => type,
    )
    qos === nothing || (msg["qos"] = qos)
    throttle_rate === nothing || (msg["throttle_rate"] = throttle_rate)
    if throttle_rate !== nothing
        msg["queue_length"] = 1
    end
    HTTP.WebSockets.send(ws, JSON3.write(msg))
end

const LATCHED_QOS = Dict(
    "reliability" => "reliable",
    "durability" => "transient_local",
    "history" => "keep_last",
    "depth" => 1,
)

# /tf is reliable + volatile. Pinning that here matters: if rosbridge sees any
# transient_local publisher on the topic it would otherwise subscribe with
# transient_local, which then drops the volatile odom -> base_link transforms.
const TF_QOS = Dict(
    "reliability" => "reliable",
    "durability" => "volatile",
    "history" => "keep_last",
    "depth" => 100,
)

function update_robot!(shared, edges, map_info)
    map_info === nothing && return
    w, h = size(map_info.grid)
    result = lookup_pose(edges, MAP_FRAME, BASE_FRAME)
    if result === nothing
        publish!(shared, status = "Map $(w)x$(h) received. Waiting for map -> base_link.")
        return
    end
    pose, tf_age = result
    status = if tf_age > TF_STALE_SEC
        "Map: $(w)x$(h) cells (STALE: tf chain $(round(tf_age, digits = 1))s old)"
    else
        "Map: $(w)x$(h) cells"
    end
    publish!(
        shared,
        robot = [robot_point(map_info, pose)],
        status = status,
    )
end

function handle_publish!(shared, edges, map_info, data)
    topic = String(jget(data, "topic", ""))
    msg = jget(data, "msg", nothing)
    msg === nothing && return map_info

    if topic == "/map"
        parsed = try
            occupancy_grid(msg)
        catch
            nothing
        end
        if parsed === nothing
            publish!(shared, status = "Ignored a /map message with an unexpected layout.")
            return map_info
        end
        map_info = parsed
        publish!(
            shared,
            grid = parsed.grid,
            status = "Map: $(size(parsed.grid, 1))x$(size(parsed.grid, 2)) cells",
        )
        update_robot!(shared, edges, map_info)
    elseif topic == "/tf" || topic == "/tf_static"
        if apply_tf!(edges, msg)
            update_robot!(shared, edges, map_info)
        end
    end
    return map_info
end

function connection_loop(shared::Shared, stop::Threads.Atomic{Bool})
    edges = Dict{String, Tuple{String, Pose, Float64}}()
    map_info = nothing

    while !stop[]
        try
            HTTP.WebSockets.open(ROSBRIDGE_URL; maxframesize = 128 * 1024 * 1024) do ws
                lock(shared.lock) do
                    shared.ws = ws
                end
                try
                    publish!(shared, status = "Connected. Subscribing to /map and /tf...")
                    subscribe(ws, "/map", "nav_msgs/msg/OccupancyGrid"; qos = LATCHED_QOS)
                    # No throttle. map -> odom and odom -> base_link arrive as
                    # separate messages; a one-deep throttle queue keeps only the
                    # newest and can drop the other link for good.
                    subscribe(ws, "/tf", "tf2_msgs/msg/TFMessage"; qos = TF_QOS)
                    subscribe(ws, "/tf_static", "tf2_msgs/msg/TFMessage"; qos = LATCHED_QOS)

                    for raw in ws
                        stop[] && break
                        data = try
                            JSON3.read(raw)
                        catch e
                            # One malformed frame shouldn't tear down and
                            # reconnect the whole websocket; skip just this
                            # message.
                            @warn "Skipping unparseable rosbridge message" exception = e
                            continue
                        end
                        data isa JSON3.Object || continue
                        op = String(jget(data, "op", ""))
                        if op == "status"
                            level = String(jget(data, "level", ""))
                            if level == "error" || level == "warning"
                                publish!(shared, status = String(jget(data, "msg", "rosbridge status")))
                            end
                            continue
                        end
                        op == "publish" || continue
                        map_info = handle_publish!(shared, edges, map_info, data)
                    end
                finally
                    lock(shared.lock) do
                        shared.ws = nothing
                    end
                end
            end
            stop[] && break
            publish!(shared, status = "rosbridge closed the connection. Retrying...")
        catch e
            stop[] && break
            publish!(
                shared,
                status = "Connection error: $(sprint(showerror, e)). Retrying...",
            )
            @error "rosbridge connection failed" exception = (e, catch_backtrace())
        end
        stop[] && break
        sleep(1)
    end
end

function main()
    fig = Figure(size = (800, 850))
    ax = Axis(fig[1, 1], title = "Go2 Map Viewer (Julia)", aspect = DataAspect())

    grid_obs = Observable(fill(NaN, 1, 1))
    heatmap!(ax, grid_obs, colormap = :grays, colorrange = (0.0, 1.0), nan_color = :transparent)

    robot_obs = Observable(Point2f[])
    scatter!(ax, robot_obs, color = :red, markersize = 14)

    status_obs = Observable("Connecting to rosbridge at $(ROSBRIDGE_URL) ...")
    Label(fig[2, 1], status_obs, tellwidth = false)

    screen = display(fig)
    opened = false
    for _ in 1:50
        if isopen(screen)
            opened = true
            break
        end
        sleep(0.02)
    end
    if !opened
        error("GLMakie did not open a window. Run this where a display is available (WSLg, or the Linux Docker compose file with X11 forwarded).")
    end

    shared = Shared(ReentrantLock(), status_obs[], grid_obs[], robot_obs[], 0, nothing)
    stop = Threads.Atomic{Bool}(false)
    # Threads.@spawn (rather than @async) lets this run on a separate OS
    # thread when the process is started with JULIA_NUM_THREADS >= 2, so
    # a large /map message being rasterized in occupancy_grid can't stall
    # GLMakie's own render loop below. With a single thread it's scheduled
    # cooperatively, same as @async would have been.
    Threads.@spawn connection_loop(shared, stop)

    seen = 0
    last_size = size(grid_obs[])
    while isopen(screen)
        status, grid, robot, generation = lock(shared.lock) do
            (shared.status, shared.grid, shared.robot, shared.generation)
        end
        if generation != seen
            if grid !== grid_obs[]
                grid_obs[] = grid
                if size(grid) != last_size
                    reset_limits!(ax)
                    last_size = size(grid)
                end
            end
            if robot != robot_obs[]
                robot_obs[] = robot
            end
            if status != status_obs[]
                status_obs[] = status
            end
            seen = generation
        end
        sleep(0.03)
    end
    stop[] = true
    # for raw in ws (connection_loop) blocks on the next message, which may
    # never arrive once the window is closed; force the socket closed so it
    # unblocks immediately instead of being abandoned mid-receive.
    ws = lock(shared.lock) do
        shared.ws
    end
    if ws !== nothing
        try
            close(ws)
        catch
        end
    end
end

main()
