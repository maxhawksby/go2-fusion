"""Dependency-free container and process resource sampling."""

import os


def _read_number(path, default=0.0):
    try:
        with open(path, encoding='utf-8') as stream:
            return float(stream.read().strip())
    except (OSError, ValueError):
        return float(default)


def _clock_ticks():
    try:
        return float(os.sysconf(os.sysconf_names['SC_CLK_TCK']))
    except (KeyError, OSError, ValueError):
        return 100.0


class ResourceSampler:
    """Sample cgroup-v2 totals and per-process resource use from /proc."""

    def __init__(self, proc_root='/proc', cgroup_root='/sys/fs/cgroup'):
        self.proc_root = proc_root
        self.cgroup_root = cgroup_root
        self.previous_container_usec = None
        self.previous_process_ticks = {}

    def sample(self, interval_s):
        interval_s = max(float(interval_s), 1e-6)
        container_usec = self._container_cpu_usec()
        if self.previous_container_usec is None:
            cpu_pct = 0.0
        else:
            cpu_pct = max(0.0, (container_usec - self.previous_container_usec)
                          / (interval_s * 1_000_000.0) * 100.0)
        self.previous_container_usec = container_usec
        memory_mb = self._container_memory_bytes() / (1024.0 * 1024.0)
        return cpu_pct, memory_mb, self._processes(interval_s)

    def _container_cpu_usec(self):
        path = os.path.join(self.cgroup_root, 'cpu.stat')
        try:
            with open(path, encoding='utf-8') as stream:
                for line in stream:
                    key, value = line.split()
                    if key == 'usage_usec':
                        return float(value)
        except (OSError, ValueError):
            return 0.0
        return 0.0

    def _container_memory_bytes(self):
        return _read_number(os.path.join(self.cgroup_root, 'memory.current'))

    def _processes(self, interval_s):
        rows = []
        try:
            entries = os.listdir(self.proc_root)
        except OSError:
            return rows
        active = set()
        for entry in entries:
            if not entry.isdigit():
                continue
            pid = int(entry)
            active.add(pid)
            process = self._process(pid, interval_s)
            if process is not None:
                rows.append(process)
        self.previous_process_ticks = {
            pid: ticks for pid, ticks in self.previous_process_ticks.items() if pid in active
        }
        return sorted(rows, key=lambda row: row['process_memory_mb'], reverse=True)

    def _process(self, pid, interval_s):
        root = os.path.join(self.proc_root, str(pid))
        try:
            with open(os.path.join(root, 'stat'), encoding='utf-8') as stream:
                stat = stream.read().split()
            ticks = float(stat[13]) + float(stat[14])
            with open(os.path.join(root, 'cmdline'), 'rb') as stream:
                command = stream.read().replace(b'\0', b' ').decode(errors='replace').strip()
            memory_kb = 0.0
            with open(os.path.join(root, 'status'), encoding='utf-8') as stream:
                for line in stream:
                    if line.startswith('VmRSS:'):
                        memory_kb = float(line.split()[1])
                        break
        except (OSError, ValueError, IndexError):
            return None
        previous = self.previous_process_ticks.get(pid)
        self.previous_process_ticks[pid] = ticks
        process_cpu = 0.0 if previous is None else max(
            0.0, (ticks - previous) / _clock_ticks() / interval_s * 100.0
        )
        name = _friendly_name(command) or f'pid-{pid}'
        return {
            'pid': pid,
            'name': name,
            'process_cpu_pct': round(process_cpu, 3),
            'process_memory_mb': round(memory_kb / 1024.0, 3),
        }


def _friendly_name(command):
    if not command:
        return ''
    known = (
        'frontier_explorer', 'slam_toolbox', 'controller_server', 'planner_server',
        'go2_driver_node', 'lidar_to_pointcloud', 'object_detection', 'scoreboard',
        'gzserver', 'ruby', 'python',
    )
    lowered = command.lower()
    for name in known:
        if name in lowered:
            return name
    return os.path.basename(command.split()[0])
