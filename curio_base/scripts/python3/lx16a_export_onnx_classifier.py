#!/usr/local/bin/python3
# 
# coding: latin-1
# 

import math
import pandas as pd
import numpy as np
import sys

# scikit-learn
from sklearn.tree import DecisionTreeClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import joblib

# Raw data produced by servo and encoder 
RAW_DATA_FILENAME = "./data/lx16a_raw_data_05.csv"

# File locations for persisted ML models
CLASSIFIER_FILENAME = "./data/lx16a_tree_classifier.joblib"
REGRESSOR_FILENAME  = "./data/lx16a_tree_regressor.joblib"

if __name__ == '__main__':
    print('lx16a_export_onnx_classifier')

    # Print the version of Python being run
    print(sys.version)

    # Load classifier
    classifier = joblib.load(CLASSIFIER_FILENAME)

