"""State and metric accumulation shared by live recording and bag replay."""

from dataclasses import dataclass, field
import math


WAITING = 'WAITING'
RECORDING = 'RECORDING'
FINALIZING = 'FINALIZING'
COMPLETE = 'COMPLETE'
ERROR = 'ERROR'


@dataclass
class RunAccumulator:
    run_id: str
    source: str = 'oak'
    run_label: str = ''
    code_version: str = 'unknown'
    source_hash: str = ''
    configuration: dict = field(default_factory=dict)
    max_odom_step_m: float = 1.0
    state: str = WAITING
    start_time_s: float = None
    latest_time_s: float = None
    end_reason: str = ''
    distance_m: float = 0.0
    known_area_m2: float = 0.0
    completeness_pct: float = None
    accuracy_error_cm: float = None
    recoveries: int = 0
    collisions: int = 0
    safety_stops: int = 0
    human_touches: int = 0
    cpu_pct: float = 0.0
    memory_mb: float = 0.0
    warnings: list = field(default_factory=list)
    timeline: list = field(default_factory=list)
    resources: list = field(default_factory=list)
    latest_map: object = None
    _previous_xy: tuple = None
    _goal_sequences: set = field(default_factory=set)
    _goal_outcomes: dict = field(default_factory=dict)
    _seen_events: set = field(default_factory=set)

    def start(self, timestamp_s):
        if self.state == WAITING:
            self.state = RECORDING
            self.start_time_s = float(timestamp_s)
            self.latest_time_s = float(timestamp_s)

    def elapsed_s(self):
        if self.start_time_s is None or self.latest_time_s is None:
            return 0.0
        return max(0.0, self.latest_time_s - self.start_time_s)

    def advance_time(self, timestamp_s):
        timestamp_s = float(timestamp_s)
        if self.start_time_s is None:
            self.start(timestamp_s)
        self.latest_time_s = max(self.latest_time_s or timestamp_s, timestamp_s)

    def handle_event(self, event_type, goal_sequence, timestamp_s, detail=''):
        stamp_key = round(float(timestamp_s), 9)
        key = (str(event_type), int(goal_sequence), stamp_key, str(detail))
        if key in self._seen_events:
            return False
        self._seen_events.add(key)
        if event_type == 'exploration_started':
            self.start(timestamp_s)
        elif self.state == WAITING:
            self.start(timestamp_s)
            self._warn_once('recording began from an event without exploration_started')
        self.advance_time(timestamp_s)
        self.timeline.append({
            'time_s': round(self.elapsed_s(), 3),
            'kind': 'event',
            'event_type': str(event_type),
            'goal_sequence': int(goal_sequence),
            'detail': str(detail),
            'distance_m': round(self.distance_m, 4),
            'known_area_m2': round(self.known_area_m2, 4),
            'completeness_pct': self._rounded_or_none(self.completeness_pct),
            'goals_sent': len(self._goal_sequences),
            'goals_reached': self._outcome_count('goal_reached'),
            'goals_failed': self._outcome_count('goal_failed'),
            'recoveries': self.recoveries,
            'collisions': self.collisions,
            'human_touches': self.human_touches,
        })
        sequence = int(goal_sequence)
        if event_type == 'goal_sent':
            self._goal_sequences.add(sequence)
        elif event_type in ('goal_reached', 'goal_failed', 'goal_cancelled'):
            self._goal_sequences.add(sequence)
            self._goal_outcomes.setdefault(sequence, event_type)
        elif event_type == 'recovery_started':
            self.recoveries += 1
        elif event_type == 'sensor_stale':
            self.safety_stops += 1
        elif event_type == 'exploration_completed':
            self.end_reason = detail or 'exploration complete'
        elif event_type == 'exploration_stopped':
            self.end_reason = detail or 'exploration stopped'
        return True

    def update_odom(self, x, y, timestamp_s):
        if self.state == WAITING:
            return
        self.advance_time(timestamp_s)
        point = (float(x), float(y))
        if self._previous_xy is not None:
            step = math.hypot(point[0] - self._previous_xy[0], point[1] - self._previous_xy[1])
            if step <= self.max_odom_step_m:
                self.distance_m += step
            else:
                self._warn_once(
                    f'ignored odometry discontinuity larger than {self.max_odom_step_m:.2f} m'
                )
        self._previous_xy = point

    def update_map(self, grid, timestamp_s, completeness=None):
        if self.state == WAITING:
            return
        self.advance_time(timestamp_s)
        self.latest_map = grid
        self.known_area_m2 = grid.known_area_m2
        if completeness is not None:
            self.completeness_pct = float(completeness)

    def add_resource_sample(self, timestamp_s, cpu_pct, memory_mb, processes=None):
        if self.state == WAITING:
            return
        self.advance_time(timestamp_s)
        self.cpu_pct = float(cpu_pct)
        self.memory_mb = float(memory_mb)
        self.resources.append({
            'time_s': round(self.elapsed_s(), 3),
            'cpu_pct': round(self.cpu_pct, 3),
            'memory_mb': round(self.memory_mb, 3),
            'processes': processes or [],
        })

    def sample_timeline(self):
        if self.state == WAITING:
            return
        self.timeline.append({
            'time_s': round(self.elapsed_s(), 3),
            'kind': 'sample',
            'event_type': '',
            'goal_sequence': '',
            'detail': '',
            'distance_m': round(self.distance_m, 4),
            'known_area_m2': round(self.known_area_m2, 4),
            'completeness_pct': self._rounded_or_none(self.completeness_pct),
            'goals_sent': len(self._goal_sequences),
            'goals_reached': self._outcome_count('goal_reached'),
            'goals_failed': self._outcome_count('goal_failed'),
            'recoveries': self.recoveries,
            'collisions': self.collisions,
            'human_touches': self.human_touches,
        })

    def mark_collision(self):
        self.collisions += 1

    def mark_human_touch(self):
        self.human_touches += 1

    def finalize(self, reason='', error=False):
        if self.state in (COMPLETE, ERROR):
            return False
        self.state = ERROR if error else COMPLETE
        if reason:
            self.end_reason = reason
        elif not self.end_reason:
            self.end_reason = 'run finalized'
        return True

    def begin_finalizing(self):
        if self.state in (WAITING, RECORDING):
            self.state = FINALIZING
            return True
        return False

    def summary(self):
        return {
            'schema_version': 1,
            'run_id': self.run_id,
            'source': self.source,
            'run_label': self.run_label,
            'code_version': self.code_version,
            'source_hash': self.source_hash,
            'configuration': dict(self.configuration),
            'state': self.state,
            'duration_s': round(self.elapsed_s(), 3),
            'distance_m': round(self.distance_m, 4),
            'known_area_m2': round(self.known_area_m2, 4),
            'completeness_pct': self._rounded_or_none(self.completeness_pct),
            'accuracy_error_cm': self._rounded_or_none(self.accuracy_error_cm),
            'goals_sent': len(self._goal_sequences),
            'goals_reached': self._outcome_count('goal_reached'),
            'goals_failed': self._outcome_count('goal_failed'),
            'goals_cancelled': self._outcome_count('goal_cancelled'),
            'recoveries': self.recoveries,
            'collisions': self.collisions,
            'safety_stops': self.safety_stops,
            'human_touches': self.human_touches,
            'cpu_peak_pct': max((row['cpu_pct'] for row in self.resources), default=0.0),
            'memory_peak_mb': max((row['memory_mb'] for row in self.resources), default=0.0),
            'end_reason': self.end_reason,
            'warnings': list(self.warnings),
        }

    def _outcome_count(self, outcome):
        return sum(value == outcome for value in self._goal_outcomes.values())

    def _warn_once(self, warning):
        if warning not in self.warnings:
            self.warnings.append(warning)

    @staticmethod
    def _rounded_or_none(value):
        return None if value is None or not math.isfinite(float(value)) else round(float(value), 4)
