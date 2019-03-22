## Index
* [how to write an basic CMakeList.txt](./how_to_write_an_cmakelist.md)
* [how to find an cmake package](./how_to_find_an_cmake_package.md)
* [how to install an config-package](./how_to_install_an_config-package.md)

## execrise
Write an c++ program to do curve interpolation by utilizing the function in package `install_package_demo`.
* Requirements:
  1. Clone this repo and checkout to `execrise` branch.
  2. Install package `install_package_demo` and observe the install locations.
  3. Write your own package to accomplish interpolation task by finding package `install_package_demo`
  4. Put your generated executable target in `cmake_tutorial/result` folder. when execute `./your_target`, your program will read the file `ctrlp.csv` already existing in `result` folder and output the interpolation result in a file named `curve.csv` in the same folder.
  5. Add your executable target in git and push to remote.
  6. If you have any problems or advise, please write an issue.
* Useful link:
  1. [git operation](https://www.liaoxuefeng.com/wiki/0013739516305929606dd18361248578c67b8067c8c017b000)
  2. [c++ file operation](https://blog.csdn.net/qq_37503115/article/details/79376743)
  3. [B-Spline related](http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html)
* Interpolation example
````
 // interpolate 4 points:
tinyspline::BSpline spline(4);

// input 4 control points: (0.0, 0.0), (10.0, 10.0), (30.0, -10.0), (50.0, 60.0);
double points[8] = {0.0, 0.0, 10.0, 10.0, 30.0, -10.0, 50.0, 60.0};
std::vector<double> control_points(points, points + 8);
spline.setControlPoints(control_points);

// evaluate:
std::vector<double> x;
std::vector<double> y;
for(double t = 0.0; t < 1; t+= 0.001) {
  x.push_back(spline.eval(t).result().at(0));
  y.push_back(spline.eval(t).result().at(1));
}
````
