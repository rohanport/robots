include("physics.jl")

using RxInfer

struct HungerChangeNode end # Dummy structure just to make Julia happy

struct HungerChangeNodeMeta end

@node HungerChangeNode Deterministic [ out, fp, p, action ]

struct HungerChangeMeta{R}
    rng      :: R
    nsamples :: Int # Number of samples used in approximation
end

# Rule for outbound message on `out` edge given inbound message on `fp`, `p` and `action` edges
@rule HungerChangeNode(:out, Marginalisation) (
    m_fp::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily, 
    m_action::MultivariateNormalDistributionsFamily,
    meta::HungerChangeMeta
) = begin
    p_m, p_v = mean_cov(m_p)
    action_m, action_v = mean_cov(m_action)
    next_p = MvNormalMeanCovariance(p_m + action_m, p_v + action_v)
    return HungerChangeDistribution(m_fp, next_p, meta)
end 

# Rule for outbound message on `fp` edge given inbound message on `out`, `p`, and `action` edges
@rule HungerChangeNode(:fp, Marginalisation) (
    m_out::UnivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily,
    m_action::MultivariateNormalDistributionsFamily,
    meta::HungerChangeMeta
) = begin
    return vague(MvNormalMeanCovariance, 2) # Can't predict fp 
end

# Rule for outbound message on `p` edge given inbound message on `out`, `fp` and `action` edges
@rule HungerChangeNode(:p, Marginalisation) (
    m_out::UnivariateNormalDistributionsFamily, 
    m_fp::MultivariateNormalDistributionsFamily,
    m_action::MultivariateNormalDistributionsFamily,
    meta::HungerChangeMeta
) = begin
    return vague(MvNormalMeanCovariance, 2) # Can't predict p 
end

# Rule for outbound message on `action` edge given inbound message on `out`, `fp` and `p` edges
@rule HungerChangeNode(:action, Marginalisation) (
    m_out::UnivariateNormalDistributionsFamily, 
    m_fp::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily,
    meta::HungerChangeMeta
) = begin
    hunger_growth = clamp(-1.0 * mean(m_out), -1.0, 1.0)

    fp_m = mean(m_fp)
    p_m = mean(m_p)

    unit_vector_to_food = clamp!(fp_m - p_m, -1.0, 1.0)

    return MvNormalMeanCovariance(hunger_growth * unit_vector_to_food, diageye(2))  
end 

@marginalrule HungerChangeNode(:fp_p_action) (
    m_out::UnivariateNormalDistributionsFamily, 
    m_fp::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily, 
    m_action::MultivariateNormalDistributionsFamily
) = begin
    return (out = m_out, fp = m_fp, p = m_p, action = m_action,)
end


function HungerChangeDistribution(
    m_fp::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily,
    meta::HungerChangeMeta
)
    fp_m = mean(m_fp)
    fp_v = var(m_fp)
    fp_x_samples = rand(meta.rng, NormalMeanVariance(fp_m[1], fp_v[1]), meta.nsamples)
    fp_y_samples = rand(meta.rng, NormalMeanVariance(fp_m[2], fp_v[2]), meta.nsamples)
    
    p_m = mean(m_p)
    p_v = var(m_p)
    p_x_samples = rand(meta.rng, NormalMeanVariance(p_m[1], p_v[1]), meta.nsamples)
    p_y_samples = rand(meta.rng, NormalMeanVariance(p_m[2], p_v[2]), meta.nsamples)

    hunger_change_samples = map(
        (fpx, fpy, px, py) -> change_in_hunger([fpx, fpy], [px, py]), 
        fp_x_samples, fp_y_samples, p_x_samples, p_y_samples
    )

    hunger_change_dist = fit_mle(Normal, hunger_change_samples)
    
    return NormalMeanVariance(mean(hunger_change_dist), var(hunger_change_dist))
end 