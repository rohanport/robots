using RxInfer, LinearAlgebra

struct NextPositionNode end # Dummy structure just to make Julia happy

@node NextPositionNode Deterministic [ out, p, o, tv ]

# Rule for outbound message on `out` edge given inbound message on `p`, `o` and `tv` edges
@rule NextPositionNode(:out, Marginalisation) (
    m_p::MultivariateNormalDistributionsFamily, 
    m_o::UnivariateNormalDistributionsFamily, 
    m_tv::UnivariateNormalDistributionsFamily
) = begin
    tv_mean = mean(m_tv)
    tv_var = var(m_tv)
    o_mean = mean(m_o)
    p_mean = mean(m_p)
    p_var = var(m_p)
    dir = [cos(o_mean), sin(o_mean)]
    out_mean = p_mean + (tv_mean * dir)
    out_var = p_var + (tv_var * dir)

    return MvNormalMeanCovariance(out_mean, out_var)
end 

# Rule for outbound message on `p` edge given inbound message on `out`, `o`, and `tv` edges
@rule NextPositionNode(:p, Marginalisation) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_o::UnivariateNormalDistributionsFamily,
    m_tv::UnivariateNormalDistributionsFamily
) = begin
    tv_mean = mean(m_tv)
    tv_var = var(m_tv)
    o_mean = mean(m_o)
    out_mean = mean(m_out)
    out_var = var(m_out)
    dir = [cos(o_mean), sin(o_mean)]
    p_mean = out_mean - (tv_mean * dir)
    p_var = out_var + (tv_var * dir)

    return MvNormalMeanCovariance(p_mean, p_var) 
end

# Rule for outbound message on `o` edge given inbound message on `out`, `p` and `tv` edges
@rule NextPositionNode(:o, Marginalisation) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily,
    m_tv::UnivariateNormalDistributionsFamily
) = begin
    out_mean = mean(m_out)
    out_var = var(m_out)
    p_mean = mean(m_p)
    p_var = var(m_p)

    delta = out_mean - p_mean
    o_mean = atan(delta[2], delta[1])
    o_var = norm(out_var) + norm(p_var)
    
    return NormalMeanVariance(o_mean, o_var) 
end

# Rule for outbound message on `tv` edge given inbound message on `out`, `p` and `o` edges
@rule NextPositionNode(:tv, Marginalisation) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily, 
    m_o::UnivariateNormalDistributionsFamily
) = begin
    out_mean = mean(m_out)
    out_var = var(m_out)
    p_mean = mean(m_p)
    p_var = var(m_p)

    delta = out_mean - p_mean
    tv_mean = norm(delta)
    tv_var = norm(out_var) + norm(p_var)
    
    return NormalMeanVariance(tv_mean, tv_var) 
end 

@marginalrule NextPositionNode(:p_o_tv) (
    m_out::MultivariateNormalDistributionsFamily, 
    m_p::MultivariateNormalDistributionsFamily, 
    m_o::UnivariateNormalDistributionsFamily, 
    m_tv::UnivariateNormalDistributionsFamily
) = begin
    return (out = m_out, p = m_p, o = m_o, tv = m_tv,)
end
