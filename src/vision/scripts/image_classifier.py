#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import csv
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt  
from sklearn.svm import SVC   
from sklearn import svm   
from sklearn.metrics import confusion_matrix, classification_report, accuracy_score
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.model_selection import train_test_split




if __name__ == "__main__":

    X = []
    Y = []

    with open("data.csv", "r") as file:
        reader = csv.reader(file)
        next(reader)

        for row in reader:

            angle1 = float(row[0])
            angle2 = float(row[1])
            angle3 = float(row[2])
            angle4 = float(row[3])
            angle5 = float(row[4])
            label = row[5]

            X.append(angle1)
            X.append(angle2)
            X.append(angle3)
            X.append(angle4)
            X.append(angle5)
            Y.append(label)

    X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state=42)


    sc = StandardScaler()
    X_train = sc.fit_transform(X_train)
    X_test = sc.transform(X_test)


    classifier = svm.SVC()
    classifier.fit(X_train, Y_train)
    predictions = classifier.predict(X_test)

    accuracy = accuracy_score(Y_test, predictions)
    print("Accuracy:", accuracy)