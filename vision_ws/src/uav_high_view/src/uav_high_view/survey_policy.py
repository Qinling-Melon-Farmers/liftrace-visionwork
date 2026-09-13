"""Navigation-hint thresholds, separate from low-altitude delivery admission."""
from dataclasses import dataclass
from .core import Config


@dataclass(frozen=True)
class SurveyPolicy:
    candidate_min_streak: int = 1
    min_interval_ns: int = 100000000
    min_span_ns: int = 200000000
    max_uncertainty_m: float = .45
    direct_descent: bool = True
    descent_radius_m: float = 1.5
    descent_max_candidates: int = 25

    def __post_init__(self):
        if (type(self.candidate_min_streak) is not int or not 1<=self.candidate_min_streak<=3
                or type(self.direct_descent) is not bool
                or not .25<=self.max_uncertainty_m<=.5
                or not 0<self.descent_radius_m<=2.
                or type(self.descent_max_candidates) is not int or not 1<=self.descent_max_candidates<=25
                or not 50000000<=self.min_interval_ns<=250000000
                or not 2*self.min_interval_ns<=self.min_span_ns<=500000000):
            raise ValueError('invalid survey-only policy')
        self.catalog_config('camera_init',600.)

    def catalog_config(self,frame,timeout):
        return Config(frame=frame,hint_ttl_ns=int(timeout*1e9),
                      min_interval_ns=self.min_interval_ns,min_span_ns=self.min_span_ns,
                      max_uncertainty_m=self.max_uncertainty_m)
