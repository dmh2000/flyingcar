# -*- coding: utf-8 -*-
import math
import numpy as np


# sum elements of an array
def Sum(a):
    return reduce(lambda x, y: x + y, a, 0.0)


# compute mean (average) of elements of an array
def Mean(a):
    s = Sum(a)
    avg = s / len(a)
    return avg


def Median(a):
    list.sort(a)
    i = len(a)
    if (i % 2) == 0:
        # even
        m = (a[i / 2] + a[i / (2 + 1)]) / 2.0
    else:
        m = a[i / 2]
    return m


# z score for a value
# (value - mean) / standard deviation
def Z(x, mu, sigma):
    return (float(x) - float(mu)) / float(sigma)


# value from z score
# (zscore * standard deviation) + mean
def D(z, mu, sigma):
    return z * sigma + mu


# compute variance of a population
def VarianceP(mu, a):
    # compute deviation squared of each element
    var = map(lambda x: (x - mu) ** 2, a)
    return sum(var) / len(var)


# compute variance of a sample
def VarianceS(mu, a):
    # compute deviation squared of each element
    var = map(lambda x: (x - mu) ** 2, a)
    return sum(var) / (len(var) - 1)


# numerical z-score from probability
# for a specified distribution
def Zp(p, mu, sigma):
    # get z score from inverse cdf
    z = Icdf(p)
    # solve for score
    return D(z, mu, sigma)


# standard error
def SE(sigma, n):
    return sigma / math.sqrt(n)


# compute population standard deviation
def SigmaP(mu, a):
    # compute deviation squared of each element
    v = map(lambda x: (x - mu) ** 2, a)
    # get sum of squares
    s = Sum(v)
    # get mean of sum of squares (variance)
    variance = s / len(v)
    # get square root of mean
    d = math.sqrt(variance)
    return d


# compute sample standard deviation
def SigmaS(mu, a):
    # compute deviation squared of each element
    v = map(lambda x: (x - mu) ** 2, a)
    # get sum of squares
    s = Sum(v)
    # get mean of sum of squares (variance)
    # with Bessel correction
    variance = s / (len(v) - 1)
    # get square root of mean
    d = math.sqrt(variance)
    return d


# gaussian
def Gaussian(x, u, sigma):
    s2 = sigma ** 2.0
    a = 1.0 / math.sqrt(2.0 * math.pi * s2)
    b = (-(x - u) ** 2.0) / (2.0 * s2)
    c = math.e ** b
    return a * c


# score from probability density function
def Pdf(sigma):
    a = math.e ** (-0.5 * sigma ** 2)
    b = a / math.sqrt(2 * math.pi)
    return b


# given a Z-score, what is the probability from the z-table
# cumulative distribution function using pascal series approximation
# @param 
def Cdf(z):
    sum = z
    val = z
    for i in range(1, 100):
        d = 2.0 * i + 1.0
        val = val * z * z / d
        sum += val
    r = 0.5 + (sum / math.sqrt(2.0 * math.pi)) * math.exp(-(z * z) / 2.0)
    return r


# given a Z-score, what is the probability from the z-table
# cumulative distribution function using numeric integration of cdf
def Cdfi(z):
    a = -10.0
    b = 0.0
    step = 0.01
    while (a < z):
        b += Pdf(a) * step
        a += step
    return b


# inverse cumulative distribution function
# given a percent, compute z score using newton's method
# measure is from 0 .. z
def Icdf(p):
    # use positive p and convert z to negative at end
    sign = 1.0
    if (p < 0):
        sign = -1.0
        p = math.fabs(p)

    # newton iteration error tolerance
    tol = 0.00001
    # initial guess
    q = 0.5
    # iterate until tolerance exceeded or N iterations
    for i in range(0, 20):
        d = (p - Cdf(q)) / Pdf(q)
        q = q + d
        if (math.fabs(d) < tol):
            break
    # round small values to 0
    if (math.fabs(q) < tol):
        q = 0
    return q * sign


# confidence interval Z score around the mean
# divide P by 2 to get the += interval around the mean
# add mean (0.5) + p / 2 to get one side of interval
def CIz(p):
    return Icdf(0.50 + (p / 2.0))


# compute value range around the mean for a given probability
def CIrange(p, mu, se):
    # z score around the mean
    z = CIz(p)
    # upper bound
    ub = D(z, mu, se)
    # lower bound
    lb = D(-z, mu, se)
    # return tuple of lb,ub
    return (lb, ub)


# margin of error
# 1/2 the confidence interval
def Me(p, mu, se):
    r = CIrange(p, mu, se)
    me = math.fabs(r[0] - r[1]) / 2.0
    return me


# likelihood
# alpha levels
# conventions indicating result isnot likely to occur by chance
# alpha   Z critical 
# 0.05  = 1.65
#   significant @ 0.05 if z is in this ramge
# 0.01  = 2.32
#   significant @ 0.01 if z is in this ramge
# 0.001 = 3.04
#   significant @ 0.001 if z is above this value
# critical region is above (1.0 - alpha)
# z critical value is z that is = (1.0 - alpha)
def Zstar(alpha):
    return Icdf(1.0 - alpha)


# likelihood
# alpha levels for 2 tailed
# conventions indicating result isnot likely to occur by chance
# alpha   Z critical 
# 0.05  = 1.96
#   significant @ 0.05 if z is in this ramge
# 0.01  = 2.57
#   significant @ 0.01 if z is in this ramge
# 0.001 = 3.29
#   significant @ 0.001 if z is above this value
# critical region is above (1.0 - alpha)
# z critical value is z that is = (1.0 - alpha)
def Zstar2(alpha):
    return Icdf(1.0 - (alpha / 2.0))


# after intervention
# null hypothesis H0     : no significant difference
#    result is inside critical region
# alternative hypothesis : signficant difference
#    result is not inside critical region
# can't prove null hypothesis, can only reject it


def T(u1, s1, n1, u2, s2, n2):
    n = (u1 - u2)
    d = math.sqrt((s1 / n1) + (s2 / n2))
    return n / d


def V(s1, n1, s2, n2):
    n = (s1 / n1) + (s2 / n2)
    n = n * n
    d = (pow(s1, 2.0) / ((n1 * n1) * (n1 - 1))) + (pow(s2, 2.0) / ((n2 * n2) * (n2 - 1)))
    return n / d


def Covariance(x, y):
    """ 2D covariance
    :param x: array of samples of x
    :param y: array os samples of y
    :return:  2x2 covariance matrix
    """
    u = np.mean(x)
    xi = x - u
    u = np.mean(y)
    yi = y - u
    a = np.stack((xi, yi), axis=0)
    b = a.T
    c = a.dot(b) / len(x)
    return c


def Bayes(a, b_given_a, not_b_given_not_a):
    """
    bayes rule
    a = has cancer
    b = positive result
    :param a:                 prior         has cancer
    :param b_given_a:         sensitivity   positive|has cancer
    :param not_b_given_not_a: specificity   negative|no cancer
    :return:
    """
    # has cancer * positive | cancer +
    # no  cancer * negative | not cancer
    # norm = (cancer * positive|cancer) + (no cancer * positive | no cancer
    # positive | no cancer = 1 - negative | no cancer
    p_b = (a * b_given_a) + (1.0 - a) * (1.0 - not_b_given_not_a)
    a_given_b = (a * b_given_a) / p_b
    return a_given_b
