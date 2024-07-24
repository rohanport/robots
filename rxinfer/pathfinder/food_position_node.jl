include("physics.jl")

using RxInfer

struct FoodPositionNode end # Dummy structure just to make Julia happy

@node FoodPositionNode Deterministic [ out, p, fdir, fint ]

# Rule for outbound message on `out` edge given inbound message on `p`, `fdir` and `fint` edges
@rule FoodPositionNode(:out, Marginalisation) (
    m_p::MultivariateNormalDistributionsFamily, 
    m_fdir::MultivariateNormalDistributionsFamily,
    m_fint::UnivariateNormalDistributionsFamily,
) = begin
    dist_from_food = cbrt(1/mean(m_fint))
    food_p_m = mean(m_p) + (dist_from_food * mean(m_fdir))
    dist_from_food_v = dist_from_food^2 / 4
    food_p_v = [dist_from_food_v, dist_from_food_v] + var(m_p)
    food_p_dist = MvNormalMeanCovariance(food_p_m, [food_p_v[1] 0.0; 0.0 food_p_v[2]])
    return food_p_dist
end 

# Rule for outbound message on `fp` edge given inbound message on `out`, `p`, and `action` edges
@rule FoodPositionNode(:p, Marginalisation) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_fdir::MultivariateNormalDistributionsFamily,
    m_fint::UnivariateNormalDistributionsFamily
) = begin
    return vague(MvNormalMeanCovariance, 2) # Can't predict p 
end

# Rule for outbound message on `fdir` edge given inbound message on `out`, `fp` and `action` edges
@rule FoodPositionNode(:fdir, Marginalisation) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily,
    m_fint::UnivariateNormalDistributionsFamily
) = begin
    return vague(MvNormalMeanCovariance, 2) # Can't predict p 
end

# Rule for outbound message on `action` edge given inbound message on `out`, `fp` and `p` edges
@rule FoodPositionNode(:fint, Marginalisation) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily, 
    m_fdir::MultivariateNormalDistributionsFamily
) = begin
    return vague(NormalMeanVariance) 
end 

@marginalrule FoodPositionNode(:p_fdir_fint) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily, 
    m_fdir::MultivariateNormalDistributionsFamily, 
    m_fint::UnivariateNormalDistributionsFamily
) = begin
    return (out = m_out, p = m_p, fdir = m_fdir, fint = m_fint,)
end
