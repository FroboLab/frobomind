#!/usr/bin/env python
#/****************************************************************************
# RANSAC algorithm
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
The implementation is based on the algorithm decribed at:
http://en.wikipedia.org/wiki/RANSAC
and inspired by:
http://wiki.scipy.org/Cookbook/RANSAC
and:
http://vision.ece.ucsb.edu/~zuliani/Research/RANSAC/docs/RANSAC4Dummies.pdf

input:
    data – a set of observations
    model – a model that can be fitted to data
    n:     the smallest number of points required
    k:     the number of iteration required
    t:     threshold that determines if a point fits well
    d:     number of nearby points required to assert that a model fits well
output:
    best_model – model parameters which best fit the data (or nil if no good model is found)
    best_consensus_set – data points from which this model has been estimated
    best_error – the error of this model relative to the data

Revision
2013-12-06 KJ First version
"""

# import
import numpy

class linear_least_squares_model:
    def __init__(self, input_columns, output_columns, debug=False):
        self.input_columns = input_columns
        self.output_columns = output_columns
        self.debug = debug
    def fit(self, data):
        A = numpy.vstack([data[:,i] for i in self.input_columns]).T
        B = numpy.vstack([data[:,i] for i in self.output_columns]).T
        x,resids,rank,s = scipy.linalg.lstsq(A,B)
        return x
    def get_error( self, data, model):
        A = numpy.vstack([data[:,i] for i in self.input_columns]).T
        B = numpy.vstack([data[:,i] for i in self.output_columns]).T
        B_fit = scipy.dot(A,model)
        err_per_point = numpy.sum((B-B_fit)**2,axis=1) # sum squared error per row
        return err_per_point


class ransac_algorithm():
	def __init__(self):
		pass

	def fit (self, data, model, n, k, t, d, debug=False, return_all=False):
		iteration = 0
		bestfit = None
		besterr = numpy.inf
		best_inlier_idxs = None

		# loop until k iterations
		while iteration < k:
			# draw a random sample of n points from data
		    maybe_idxs, test_idxs = random_partition(n, data.shape[0])
		    maybeinliers = data[maybe_idxs,:]
		    test_points = data[test_idxs]
		    maybemodel = model.fit(maybeinliers)
		    test_err = model.get_error( test_points, maybemodel)
		    also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
		    alsoinliers = data[also_idxs,:]
		    if debug:
		        print 'test_err.min()',test_err.min()
		        print 'test_err.max()',test_err.max()
		        print 'numpy.mean(test_err)',numpy.mean(test_err)
		        print 'iteration %d:len(alsoinliers) = %d'%(
		            iteration,len(alsoinliers))
		    if len(alsoinliers) > d:
		        betterdata = numpy.concatenate( (maybeinliers, alsoinliers) )
		        bettermodel = model.fit(betterdata)
		        better_errs = model.get_error( betterdata, bettermodel)
		        thiserr = numpy.mean( better_errs )
		        if thiserr < besterr:
		            bestfit = bettermodel
		            besterr = thiserr
		            best_inlier_idxs = numpy.concatenate( (maybe_idxs, also_idxs) )
		    iteration+=1
		if bestfit is None:
		    raise ValueError("did not meet fit acceptance criteria")
		if return_all:
		    return bestfit, {'inliers':best_inlier_idxs}
		else:
		    return bestfit


ransac = ransac_algorithm()

data = []

ransac.fit (

