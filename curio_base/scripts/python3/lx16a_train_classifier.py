#!/usr/local/bin/python3
# 
#   Software License Agreement (BSD-3-Clause)
#    
#   Copyright (c) 2019 Rhys Mainwaring
#   All rights reserved
#    
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   1.  Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#   2.  Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#   3.  Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#  
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 

''' Train the encoder classifier (Python 3 ROS independent version).
'''

import joblib
import math
import numpy as np
import pandas as pd

from sklearn.metrics import accuracy_score
from sklearn.model_selection import cross_validate
from sklearn.model_selection import train_test_split
from sklearn.pipeline import make_pipeline
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.tree import DecisionTreeClassifier

from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType

if __name__ == '__main__':
    print('Starting LX-16A classifier training')

    # Parameters
    check_accuracy_score = False
    check_cross_validation_score = False
    data_dir = "/Users/rhys/Code/robotics/curio/curio_ws/src/curio/curio_base/data"
    dataset_filename = data_dir + "/lx16a_dataset.zip"
    classifier_filename = data_dir + "/lx16a_tree_classifier.joblib"
    onnx_filename = data_dir + "/lx16a_tree_classifier.onnx"

    # Load data from CSV 
    print('Loading dataset: {}'.format(dataset_filename))
    df_data = pd.read_csv(dataset_filename, index_col=0, compression='zip')

    # Extract numpy arrays
    print('Selecting features and targets')
    df_X = df_data.iloc[:,:-1]
    df_y = df_data.iloc[:,-1:]
    X = df_X.values
    y = df_y.values.ravel()
    print("X.shape: {}, y.shape {}".format(X.shape, y.shape))

    # Create training and test datasets
    print('Creating training and test datasets')
    df_X_train, df_X_test, df_y_train, df_y_test = train_test_split(df_X, df_y, random_state=0)
    X_train = df_X_train.values
    y_train = df_y_train.values.ravel()
    X_test = df_X_test.values
    y_test = df_y_test.values.ravel()

    # Create a pipeline for the decision tree classifier
    print('Creating decision tree pipeline')
    clf_pipe = make_pipeline(
        StandardScaler(),
        DecisionTreeClassifier(random_state=0)
    )

    # Fit the whole pipeline
    print('Fitting model to training dataset')
    clf_pipe.fit(X_train, y_train)

    # Check accuracy score
    if check_accuracy_score:
        print('Calculating accuracy score...')
        score = accuracy_score(clf_pipe.predict(X_test), y_test)
        print('Accuracy score: {}'.format(score))
 
    # Cross validation score
    if check_cross_validation_score:
        print('Calculating cross validation scores...')
        cv = cross_validate(clf_pipe, X, y, cv=5)
        print('CV test score:  {}'.format(cv['test_score']))

    # Persist using joblib (better for larger models)
    print('Writing classifier: {}'.format(classifier_filename))
    joblib.dump(clf_pipe, classifier_filename, protocol=2)

    # Persist using skl2onnx
    print('Writing classifier: {}'.format(onnx_filename))
    initial_type = [('float_input', FloatTensorType([None, 4]))]
    onx = convert_sklearn(clf_pipe, initial_types=initial_type)
    with open(onnx_filename, "wb") as f:
        f.write(onx.SerializeToString())
    