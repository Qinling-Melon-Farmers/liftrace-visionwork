"""Navigation-hint thresholds, separate from low-altitude delivery admission."""
from dataclasses import dataclass
from .core import Config


@dataclass(frozen=True)
class SurveyPolicy:
    resume_survey_enabled: bool = False
    resume_overlap_m: float = .30
    resume_budget_seconds: float = 90.
    descent_wait_seconds: float = 2.
    coarse_enabled: bool = False
    coarse_min_confidence: float = .60
    coarse_uncertainty_m: float = .45
    coarse_interrupt_min_interval_ns: int = 100000000
    coarse_interrupt_max_gap_ns: int = 1000000000
    coarse_interrupt_consistency_m: float = .5
    interrupt_refined_classes: tuple = ()
    recheck_observe_seconds: float = 5.
    recheck_shift_after_seconds: float = 1.
    recheck_shift_radius_m: float = .5
    high_min_agl: float = 2.0
    high_max_agl: float = 3.0
    candidate_min_streak: int = 1
    min_interval_ns: int = 100000000
    min_span_ns: int = 200000000
    max_uncertainty_m: float = .45
    direct_descent: bool = True
    descent_radius_m: float = 1.5
    descent_max_candidates: int = 25
    survey_stall_seconds: float = 8.
    survey_progress_m: float = .15
    survey_alternative_radius_m: float = .6

    def __post_init__(self):
        if (type(self.resume_survey_enabled) is not bool or
                not 0<=self.resume_overlap_m<=1. or
                not 10<=self.resume_budget_seconds<=180. or
                not 0<=self.descent_wait_seconds<=5. or
                type(self.coarse_enabled) is not bool or
                not .0 <= self.coarse_min_confidence <= 1.0 or
                not .25 <= self.coarse_uncertainty_m <= .5 or
                not 50000000<=self.coarse_interrupt_min_interval_ns<=250000000 or
                not self.coarse_interrupt_min_interval_ns<=self.coarse_interrupt_max_gap_ns<=1000000000 or
                not .1<=self.coarse_interrupt_consistency_m<=.5 or
                not 3.<=self.recheck_observe_seconds<=15. or
                not .5<=self.recheck_shift_after_seconds<=1. or
                not .4<=self.recheck_shift_radius_m<=.6 or
                not 3.<=self.survey_stall_seconds<=20. or not .05<=self.survey_progress_m<=.3
                or not .3<=self.survey_alternative_radius_m<=1.
                or type(self.candidate_min_streak) is not int or not 1<=self.candidate_min_streak<=3
                or type(self.direct_descent) is not bool
                or not .25<=self.max_uncertainty_m<=.5
                or not 0<self.descent_radius_m<=2.
                or type(self.descent_max_candidates) is not int or not 1<=self.descent_max_candidates<=25
                or not 50000000<=self.min_interval_ns<=250000000
                or not 2*self.min_interval_ns<=self.min_span_ns<=500000000):
            raise ValueError('invalid survey-only policy')
        if (not isinstance(self.interrupt_refined_classes,(list,tuple)) or
                any(not isinstance(c,str) or not c for c in self.interrupt_refined_classes) or
                len(set(self.interrupt_refined_classes))!=len(self.interrupt_refined_classes)):
            raise ValueError('invalid refined interruption classes')
        self.catalog_config('camera_init',600.)

    def catalog_config(self,frame,timeout):
        return Config(frame=frame,hint_ttl_ns=int(timeout*1e9),high_min_agl=self.high_min_agl,high_max_agl=self.high_max_agl,
                      min_interval_ns=self.min_interval_ns,min_span_ns=self.min_span_ns,
                      max_uncertainty_m=self.max_uncertainty_m)
