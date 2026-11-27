import pytest

from scripts.audit_segment_ensemble import summarize


def row(seed, available, long_share):
    def segment(h, share):
        return dict(rmse_h=h, rmse_v=2., max_h=3*h, horizontal_sse_fraction=share)
    return dict(seed=seed, segments={'available': segment(available, 1-long_share),
                                   'outage_120_180': segment(10., long_share)})


def test_equal_run_summary_counts_and_descriptive_ranges():
    result = summarize([row(1, .5, .9), row(2, 2., .3)])
    assert result['n_runs'] == 2
    assert result['long_outage_dominates_count'] == 1
    assert result['available_h_at_most_1m_count'] == 1
    assert result['segments']['available']['rmse_h'] == dict(min=.5, median=1.25, max=2.)


def test_reject_pseudoreplication_empty_and_nonfinite():
    for rows in ([], [row(1, 1., .9), row(1, 2., .8)], [row(1, float('nan'), .9)]):
        with pytest.raises(ValueError):
            summarize(rows)
