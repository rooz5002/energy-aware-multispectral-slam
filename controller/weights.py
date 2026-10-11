"""Modality weights (paper Eq. 2):  omega_m(t) = Q_m(t) / sum_k Q_k(t), over active modalities."""

EPS = 1e-9


def weights(q: dict, active: dict) -> dict:
    """q: modality -> Q; active: modality -> bool. Inactive modalities get weight 0.

    Q can be slightly negative (the blur term is subtracted), so it is clipped at 0 here.
    If every active modality has Q <= 0 the weights fall back to equal shares.
    """
    on = [m for m in q if active.get(m, False)]
    out = {m: 0.0 for m in q}
    if not on:
        return out
    clipped = {m: max(0.0, q[m]) for m in on}
    total = sum(clipped.values())
    for m in on:
        out[m] = clipped[m] / total if total > EPS else 1.0 / len(on)
    return out


def feature_split(omega: dict, n_total: int, active: dict, n_min: int = 50) -> dict:
    """Per-modality ORB budget n_m = round(omega_m * n_total), at least n_min while the modality is active.

    The budgets always sum to n_total.
    """
    on = [m for m in omega if active.get(m, False)]
    out = {m: 0 for m in omega}
    if not on:
        return out
    if len(on) == 1:
        out[on[0]] = int(n_total)
        return out
    floor = min(n_min, n_total // len(on))
    raw = {m: max(floor, int(round(omega[m] * n_total))) for m in on}
    # give the rounding / minimum correction to the modality with the largest share
    big = max(on, key=lambda m: raw[m])
    raw[big] += n_total - sum(raw.values())
    out.update(raw)
    return out
