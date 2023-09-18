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

data = pd.read_csv("data.csv")
X = data.iloc[:, :-1].values
Y = data.iloc[:, -1].values

label_encoder = LabelEncoder()
Y = label_encoder.fit_transform(Y)

X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.1, train_size=0.9, random_state=42)

sc = StandardScaler()
X_train = sc.fit_transform(X_train)
X_test = sc.transform(X_test)

classifier = SVC()
classifier.fit(X_train, Y_train)
predictions = classifier.predict(X_test)
accuracy = accuracy_score(Y_test, predictions)
print("Accuracy:", accuracy)

'''
def angle_callback(data):
    
    angles = data.data
    angles = sc.transform([angles])
    label = classifier.predict(angles)
    print(label)



if __name__ == "__main__":
    
    rospy.init_node("Node")
    subscriber = rospy.Subscriber("/angles", Float32MultiArray, angle_callback)


    #accuracy = accuracy_score(Y_test, predictions)
    #print("Accuracy:", accuracy)
    rospy.spin()
'''