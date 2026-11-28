import pytest

from scripts.audit_crossed_initialization import summarize_crossed


def cell(seed, init, value):
    return dict(seed=seed, init_seed=init, segments={'available': {'rmse_h': value}})


def test_conditional_ranges_do_not_pool_crossed_cells():
    result = summarize_crossed([cell(1, 0, 1), cell(1, 2, 3),
                                cell(4, 0, 10), cell(4, 2, 11)])
    assert result['n_scenarios'] == 2
    assert result['n_initializations'] == 2
    assert result['n_cells'] == 4
    assert result['conditional_horizontal_rmse']['1']['available'] == dict(min=1., max=3., span=2.)
    assert result['conditional_horizontal_rmse']['4']['available']['span'] == 1.


@pytest.mark.parametrize('rows', [[], [cell(1, 0, 1)]*2,
    [cell(1, 0, 1), cell(1, 2, 2), cell(2, 0, 3)],
    [cell(1, 0, float('nan'))], [cell(1, 0, -1)]])
def test_invalid_grid_or_metrics_refused(rows):
    with pytest.raises(ValueError):
        summarize_crossed(rows)
