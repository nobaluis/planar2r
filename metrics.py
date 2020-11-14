# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 14:14:52 2020

@author: diego
"""

import numpy as np

def mse (error):
    """Compute the mse metric"""
    mse = np.square(error).mean()
    return mse

def ise (error):
    """Compute the ise metric"""
    ise = np.sum(np.square(error))
    return ise


def iae (error):
    """Compute the iae metric"""
    iae = np.sum(np.abs(error))
    return iae
    
def itae (t, error):
    """Compute the itae metric"""
    itae = np.sum(t @ np.abs(error))
    return itae

def itse (t, error):
    """Compute the itse metric"""
    itae = np.sum(t @ np.square(error))
    return itae
    
    
    