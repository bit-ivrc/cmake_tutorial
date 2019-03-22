#!/usr/bin/env python
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
pwd = os.getcwd()


data = pd.read_csv(pwd+"/curve.csv")
x = data["x"]
y = data["y"]

data2 = pd.read_csv(pwd+"/ctrlp.csv")
x2 = data2["x"]
y2 = data2["y"]

plt.plot(x,y, x2, y2)

plt.show()

