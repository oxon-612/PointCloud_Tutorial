[TOC]

## Chapter8   Feature

We have introduced Keypoints and Feature in Chapter4, which can help us detect point cloud, reconstruct point cloud, and SLAM. Better essential information, higher guesstimate. Then we can distinguish between the objects in the point cloud. Among them, we also call Keypoints Feature points. Before calculating Feature points,  we need to extract/detect Feature points/Keypoints, then we descript them (see in Chapter4). Finally, we can match them to operate registration and reconstruction.

In this Chapter, we will jump out of the PCL range, start with the 2D image Feature points, and then gradually expand to the Feature points in the point cloud. Besides, algorithms for processing Feature points are also divided into traditional methods and methods based on deep learning, which we will mention in this Chapter.



### 8-1：Feature in image



#### Detection

Feature points are points that contain feature information. They are usually called interest points and Keypoints in research, as shown in the figure below:

<img src="pics\1.png" alt="image-20200925220433040" style="zoom:47%;" />

*Fig.8.1.1:   Feature points[^1][^2]*

As shown in Fig.8.1.1, the edge points marked by different colors in (a) and (b) and the points in a certain area are called Feature points, which are the Keypoints we use to capture data information.



#### Description

Describing Feature points is often based on the Feature points we extracted in the previous step and their neighborhoods for information processing, such as obtaining a histogram (PFH, FPFH, VFH introduced in Chapter4) or obtaining a vector (Normal introduced in Chapter4).

Fig.8.1.2 is the histogram of the description based on the Feature points in the image:

<img src="pics\2.png" alt="image-20200926081341145" style="zoom:39%;" />

*Fig.8.1.2:   The description of Feature points[^3]*



#### Matching

After obtaining the description of the Feature point, we can match or register the point cloud by comparing the similarity between the two features. As shown below:

<img src="pics\3.png" alt="image-20201024112213381" style="zoom:47%;" />

*Fig.8.1.3:   Matching[^4]*

Match the features (red line) in the two images at the top of Fig.8.1.3, go through a series of operations such as registration, and get the matching result at the bottom.

This kind of thinking is also very common in our daily life, such as panoramic pictures taken from mobile phones.



#### Application

The feature points extracted from the image have a wide range of uses, such as in the field of SLAM (Simultaneous Localization And Mapping): we select feature points in the image and use the feature points for positioning and mapping. SLAM will get the depth information of the image and the position of the camera during the processing. According to this information, the feature points in the 2D image can be mapped to the 3D points, as shown in Fig.8.1.4.

See more information in  https://webdiis.unizar.es/~raulmur/orbslam/

![image-20201025094341270](pics\4.png)

*Fig.8.1.4:   SLAM[^5]*



#### Harris 2D

After understanding the basics of feature points in images, let us learn a feature point extraction algorithm —— Harris. The feature point algorithm can be extended to three dimensions.

Harris' general idea is derived from patch matching, which is the matching of small areas in the image. As shown below:

![image-20200926110136599](pics\5.png)

*Fig.8.1.5:   Patch matching[^6]*

In Line 1, we can easily see that one of the four green areas in (b) matches the red area in (a). However, for the green area in (d) in Line 2, it is difficult for us to judge which one is the matching area of (c) because their content is too similar. There is no recognition, that is, there is no apparent information, so the corner/junction area is selected when considering matching. As shown below:

![image-20200926111759313](pics\6.png)

*Fig.8.1.6:   The explanation of corner*

We placed a right angle in the picture. When the orange patch moves up, down, left, and right at the position shown in (a), the information inside the patch, such as the color (marked as Intensity here), does not change; When a small amount shifts up and down the position shown in (b), the position of the black line in the orange patch will change, causing the internal color information to change, but it will not be shifted in the left and right directions; move up and down in the position shown in (c), there will be changes in the internal information of the patch when there is a small shift from left to right. (c) is the corner area we are looking for, which contains rich neighborhood information.

In summary, a useful feature point extraction location is that as the sliding window (such as the orange patch in Fig.8.1.6) moves, the information changes all the time.

The following is a mathematical analysis: given an image, for a point $(x, y)$ in a small area $\Omega$ in the image, use [$\Delta$x, $\Delta$y] to realize the sliding of the small area. The change in intensity of the small area after sliding compared to before is:
$$
E(\Delta x, \Delta y)=\sum_{x, y \in \Omega} w(x, y)[I(x+\Delta x, y+\Delta y)-I(x, y)]^{2}
$$
Among them, $w(x, y)$ is a window function, used to adjust the weight of different areas, here the value in the area $\Omega$ is 1, otherwise the value is 0. Since ${x, y \in \Omega}$, so $w(x, y) = 1$. If the calculated intensity change of a small area is large, it can indicate that this area is a good feature point extraction location.

**Q: How to choose [$\Delta$x，$\Delta$y]?** 

A: The smaller the better, the smaller it can be approximated as a gradient problem. Here, $w(x, y) = 1$, the derivation of the above formula (1) is as follows:
$$
\begin{aligned}
E(\Delta x, \Delta y) &=\sum_{x, y \in \Omega}(I(x+\Delta x, y+\Delta y)-I(x, y))^{2} \\
& \approx \sum_{x, y \in \Omega}\left(I(x, y)+\Delta x I_{x}+\Delta y I_{y}-I(x, y)\right)^{2} \\
&=\sum_{x, y \in \Omega} \Delta x^{2} I_{x}^{2}+2 \Delta x \Delta y I_{x} I_{y}+\Delta y^{2} I_{y}^{2} \\
&=\sum_{x, y \in \Omega}\left[\begin{array}{cc}
\Delta x & \Delta y
\end{array}\right]\left[\begin{array}{cc}
I_{x}^{2} & I_{x} I_{y} \\
I_{x} I_{y} & I_{y}^{2}
\end{array}\right]\left[\begin{array}{c}
\Delta x \\
\Delta y
\end{array}\right]
\\
&=\left[\begin{array}{ll}
\Delta x & \Delta y
\end{array}\right]\left(\sum_{x, y \in \Omega}\left[\begin{array}{cc}
I_{x}^{2} & I_{x} I_{y} \\
I_{x} I_{y} & I_{y}^{2}
\end{array}\right]\right)\left[\begin{array}{l}
\Delta x \\
\Delta y
\end{array}\right]
\end{aligned}
$$

> Here we need to add mathematical knowledge, such as Taylor expansion, derivative, gradient and other concepts.

Among them, the matrix $\left[\begin{array}{cc}
I_{x}^{2} & I_{x} I_{y} \\
I_{x} I_{y} & I_{y}^{2}
\end{array}\right]$ is recorded as $M$, that is, the intensity of the covariance matrix composed of the first derivative in the x and y directions, and the intensity derivative $I_i$ of the point $(x_i, y_i)$ is recorded as $[I_{x_i}, I_{y_i}]^T$. We can analyze the distribution of data by analyzing the covariance matrix.

Fig.8.1.7 reflects the relationship between the derivative and the area change. For the edge case, when we move in the x-axis direction, the intensity will change, because the color changes from black to grey and there is no change when moving in the y-axis direction, so the y-axis derivative is 0. For flat case, the color in this area does not change no matter how to move. Therefore, the derivatives of the intensity on the x-axis and the y-axis are both 0. For corner, whether it is moving on the x-axis or y-axis, the intensity will change, and there is an intersection. If the x and y axis derivatives of the intersection point are non-zero, then this point can be used as a feature point. Therefore, the characteristic point can be regarded as a position with a larger derivative in both directions.

![image-20201025104854675](pics\7.png)

*Fig.8.1.7:   The relationship between derivative and area[^6]*

But we don't need to judge the magnitude of the derivative of each axis every time. We can directly analyze the eigenvalues $\lambda_1, \lambda_2$ of the covariance matrix $M$. Both $\lambda_1$ and $\lambda_2$ are very large, indicating that the distribution in the two eigenvector directions is very scattered and uniform. So it can be judged like this:

<img src="pics\8.png" alt="image-20201025105519793" style="zoom:47%;" />

*Fig.8.1.8:   How to judge corner[^6]*

- $\lambda_1, \lambda_2$ are very large, indicating that it is a corner area;
- One of $\lambda_1, \lambda_2$ is much larger than the other, indicating that it is an edge area;
- $\lambda_1, \lambda_2$ are very small, indicating a smooth flat area.

Therefore, we can determine the degree of size by setting the threshold $\lambda$. There are now two eigenvalues, but we don’t have to compare both eigenvalues. The common method is to select the smallest eigenvalue $min(\lambda_1,\lambda_2)$ (also known as the response function) with a certain threshold for comparison.

There are many types of response functions. $min(\lambda_1,\lambda_2)$ is one of them. Let us briefly list the commonly used response functions:

<img src="pics\9.png" alt="image-20200927083609904" style="zoom: 39%;" />

*Fig.8.1.9:   Response function*

**Example**

The following figure is the process diagram of Harris corner detection for two images:

- ① is the two original images.
- ② is the response image obtained by obtaining each point and the neighborhood calculation (the redder, the larger).
- ③ is the extracted Harris corner point.
- ④ is the extraction effect after NMS (see in Chapter7), because ③ got too many Harris corner points, so it is filtered and simplified by NMS.

![image-20200926160543490](pics\11.png)

*Fig.8.1.10:   Harris in image (a)[^7]*

Fig.8.1.11 is the effect of drawing the Harris corner point obtained in Fig.8.1.10 on the original image (red points):

![image-20200926161022688](pics\10.png)

*Fig.8.1.11:   Harris in image (b)[^7]*



### 8-2：Feature in point cloud

We mainly introduced the detection and extraction for Harris in the image in section 8-1. Harris can also be expanded into three-dimensional space. This section we will talk about Feature in the point cloud. Firstly, let us learn the application of 3D Feature:

- Registration: The traditional method ICP can also be used for registration. However, it needs a relatively good solution: rotation matrix $R$ and translation vector $T$. Besides, the high overlap rate between two point clouds is also very crucial;

  ![image-20200927085255033](pics\12.png)

  *Fig.8.2.1:   Registration[^8]*

- Pose estimation: Given a model, how to estimate the location and pose through feature;

  ![image-20200927090028527](pics\13.png)

  *Fig.8.2.2:   Pose estimation[^9]*

- Digital human drive: The facial expressions —— feature points are obtained by shooting, and transferred to the three-dimensional digital human to drive the three-dimensional digital human. It is applied in major live broadcast platforms, short videos. Click on the link https://youtu.be/xMgoypPBEgw to feel.

- 通过拍摄得到人的表情 —— 特征点，转移到三维数字人上，驱动三维数字人。应用在各大直播平台，短视频。点击链接感受 https://youtu.be/xMgoypPBEgw

  <img src="pics\14.png" alt="image-20200927112233541" style="zoom:47%;" />

*Fig.8.2.3:   Digital human drive[^10]*

Next, we will introduce traditional and deep learning-based feature point algorithms.



#### 8-2-1：Detection

##### Traditional method

###### Harris 3D

Similar to the derivation process of Harris 2D in images, and assumes that the point cloud itself contains intensity information. We first need to lock an area $\Omega$ in the point cloud. We can use the Fixed Radius-NN that we learned before to determine a neighboring area $\Omega$, and then for the $(x, y,z)$ in $\Omega$ , set the offset [$\Delta$x, $\Delta$y, $\Delta$z], formula $(1)$ becomes:
$$
E(\Delta x, \Delta y,\Delta z)=\sum_{x, y, z \in \Omega} w(x, y, z)[I(x+\Delta x, y+\Delta y,z+\Delta z)-I(x, y,z)]^{2}
$$
$w(x, y, z)$ is consistent with the previous definition, because x, y, and z are all in $\Omega$, so the value of this function is 1, which can be omitted and becomes $(3)$:
$$
E(\Delta x, \Delta y,\Delta z)=\sum_{x, y, z \in \Omega}[I(x+\Delta x, y+\Delta y,z+\Delta z)-I(x, y,z)]^{2}
$$
After the same formula derivation (Taylor expansion) as in the 2D case, $(4)$ is obtained:
$$
\left[\begin{array}{ll}
\Delta x & \Delta y & \Delta z
\end{array}\right]\left(\sum_{x, y, z \in \Omega}\left[\begin{array}{cc}
I_{x}^{2} & I_{x} I_{y} & I_{x} I_{z}\\
I_{x} I_{y} & I_{y}^{2} & I_{y} I_{z}\\
I_{x} I_{z} & I_{y} I_{z} & I_{z}^{2}
\end{array}\right]\right)\left[\begin{array}{l}
\Delta x \\
\Delta y \\
\Delta y
\end{array}\right]
$$
Nevertheless, calculating the derivative of each axis in the point cloud is more complicated than in the image (the discreteness of the point cloud), so we take the following method:

Suppose we want to study is the point $p(p_x,p_y,p_z)$ in the point cloud. Then in the area $\Omega$, there is point $q(q_{xi},q_{yi},q_{zi}) $, set the direction of the vector $e(e_x,e_y,e_z)$ to be the direction where the intensity changes the most. Obtain the following equation:
$$
\left(q_{x1}-p_{x}\right) e_{x}+\left(q_{y1}-p_{y}\right) e_{y}+\left(q_{z1}-p_{z}\right) e_{z}=I\left(q_{x1}, q_{y1}, q_{z1}\right)-I\left(p_{x}, p_{y}, p_{z}\right)\\
\left(q_{x2}-p_{x}\right) e_{x}+\left(q_{y2}-p_{y}\right) e_{y}+\left(q_{z2}-p_{z}\right) e_{z}=I\left(q_{x2}, q_{y2}, q_{z2}\right)-I\left(p_{x}, p_{y}, p_{z}\right)\\
...\\
\left(q_{xi}-p_{x}\right) e_{x}+\left(q_{yi}-p_{y}\right) e_{y}+\left(q_{zi}-p_{z}\right) e_{z}=I\left(q_{xi}, q_{yi}, q_{zi}\right)-I\left(p_{x}, p_{y}, p_{z}\right)\\
...
$$

Written in matrix form:
$$
\begin{array}{l}
\mathbf{q}_{xi}^{\prime T} \mathbf{e}=\Delta I_{i} \\
\mathbf{q}_{xi}^{\prime}=\left[q_{xi}^{\prime}, q_{yi}^{\prime}, q_{zi}^{\prime}\right]^{T}=\left[q_{xi}-p_{x}, q_{yi}-p_{y}, q_{zi}-p_{z}\right]^{T}
\end{array}
$$

$$
A \mathbf{e}=\mathbf{b}, A=\left[\begin{array}{ccc}
q_{x1}^{\prime} & q_{y1}^{\prime} & q_{z1}^{\prime} \\
\vdots & \vdots & \vdots \\
q_{xi}^{\prime} & q_{yi}^{\prime} & q_{zi}^{\prime} \\
\vdots & \vdots & \vdots
\end{array}\right], \mathbf{b}=\left[\begin{array}{c}
\Delta I_{1} \\
\vdots \\
\Delta I_{i} \\
\vdots
\end{array}\right]
$$

According to $(7)$, assuming that the number of points is large enough, we can get the value of $\bold e$, which is the first derivative of intensity in each direction:
$$
\mathbf{e}=\left(A^{T} A\right)^{-1} A^{T} \mathbf{b}
$$
And: 
$$
\mathbf{e} = (I_x,I_y,I_z)
$$
Therefore, we can get matrix $M$ in $(4)$.

------

We can also optimize this method:

Project $\mathbf{e}$ to a local surface instead of $\mathbf{e}$ before projection. We use an approximate plane equation to represent this local surface $ax+by+cz+d=0$, then the normal vector of the surface is:
$$
\mathbf{n}=\left[n_{x}, n_{y}, n_{z}\right]^{T}=\frac{[a, b, c]^{T}}{\left\|[a, b, c]^{T}\right\|_{2}}
$$
**Why project $\mathbf{e}$ to a local surface？**

Because the point cloud often contains some noise points, if we use $\mathbf{e}$ directly, the direction of the feature vector may be affected by the noise points and deviate. The blue arrow in the following figure represents $\mathbf{e}$, and the green arrow represents the direction after projection. It can be seen that the blue arrow is affected by the noise point on the upper right.

<img src="pics\15.png" alt="image-20201026095030517" style="zoom:66%;" />

*Fig.8.2.4:   projection*

The $\mathbf{e}^{\prime}$ after projection is: 
$$
\mathbf{e}^{\prime}=\mathbf{e}-\mathbf{n}\left(\mathbf{n}^{T} \mathbf{e}\right)=\mathbf{e}-\mathbf{n}\left(\mathbf{e}^{T} \mathbf{n}\right)
$$
Therefore, when calculating the matrix $M$, the first-order derivative used is the $\mathbf{e}^{\prime}$ projected from the local surface to reduce the sensitivity to noise points.

Finally, we get the eigenvalues $\lambda_1, \lambda_2,\lambda_3$. Regarding the response function of Harris in the point cloud, we generally do not take the smallest eigenvalue, because the direction of this eigenvalue is generally the z-axis. For scenes such as roads, the change of the z-axis direction is not big, it is easy to miss the feature point, so we use the eigenvalue in the middle as the value of the response function to compare with it.

------

**If there is no intensity in the point cloud, how we get Harris?**

Because there is no intensity information, our formula (3) becomes:
$$
E(\Delta x, \Delta y,\Delta z)=\sum_{x, y, z \in \Omega}[f(x+\Delta x, y+\Delta y,z+\Delta z)-f(x, y,z)]^{2}
$$
Among them, $f(x,y,z)=0$ represents a local plane, that is, $ax+by+cz+d=0$. According to Taylor expansion, (12) can be expressed as:
$$
\left[\begin{array}{ll}
\Delta x & \Delta y & \Delta z
\end{array}\right]\left(\sum_{x, y, z \in \Omega}\left[\begin{array}{cc}
n_{x}^{2} & n_{x} n_{y} & n_{x} n_{z}\\
n_{x} n_{y} & n_{y}^{2} & n_{y} n_{z}\\
n_{x} n_{z} & n_{y} n_{z} & n_{z}^{2}
\end{array}\right]\right)\left[\begin{array}{l}
\Delta x \\
\Delta y \\
\Delta y
\end{array}\right]
$$
Among them, $(n_x,n_y,n_z)$ is the surface normal of $ax+by+cz+d=0$. At this time, the response function is the smallest value $\lambda_3$ among the eigenvalues.

###### Harris 6D

In Harris 6D, the point cloud itself contains intensity, and we also use normal information. The covariance matrix is:
$$
M=\sum_{x, y, z \in \Omega}\left[\begin{array}{ccccc}
I_{x}^{2} & I_{x} I_{y} & I_{x} I_{z} & I_{x} n_{x} & I_{x} n_{y} & I_{x} n_{z} \\
I_{x} I_{y} & I_{y}^{2} & I_{y} I_{z} & I_{y} n_{x} & I_{y} n_{y} & I_{y} n_{z} \\
I_{x} I_{z} & I_{y} I_{z} & I_{z}^{2} & I_{z} n_{x} & I_{z} n_{y} & I_{z} n_{z} \\
n_{x} I_{x} & n_{x} I_{y} & n_{x} I_{z} & n_{x}^{2} & n_{x} n_{y} & n_{x} n_{z} \\
n_{y} I_{x} & n_{y} I_{y} & n_{y} I_{z} & n_{x} n_{y} & n_{y}^{2} & n_{y} n_{z} \\
n_{z} I_{x} & n_{z} I_{y} & n_{z} I_{z} & n_{x} n_{z} & n_{y} n_{z} & n_{z}^{2}
\end{array}\right]
$$
Choose $\lambda_4$ as the response function of Harris 6D, which is the fourth-smallest eigenvalue after sorting.

###### ISS

Harris 3D and Harris 6D are derived from Harris 2D in the image. Point cloud also has its original feature points, such as ISS.

The idea of ISS (Intrinsic Shape Signature) is:

① Keypoints/feature points often appear in areas with massive changes in points' feature;

② PCA can describe the distributions of three directions, build a covariance matrix for the neighbor of each point. Among them, PCA (Principle component analysis) is a reducing dimension through rotating the base of coordinate to make data spread into two axes.

More details in https://en.wikipedia.org/wiki/Principal_component_analysis

> ISS considered the weight when establishing the covariance matrix. The closer the point, the greater the weight.

③ Calculate the eigenvalues of the covariance matrix of the point $p_i$, get $\lambda_i^1,\lambda_i^2,\lambda_i^3$, and satisfy the following formula:
$$
\frac{\lambda_{i}^{2}}{\lambda_{i}^{1}}<\gamma_{21} \text { and } \frac{\lambda_{i}^{3}}{\lambda_{i}^{2}}<\gamma_{32}
$$
Make $\lambda_i^1>\lambda_i^2>\lambda_i^3$ to exclude planes and lines (because planes and lines may have their two eigenvalues equal).

④ Select $\lambda_i^3$ via NMS. Because the principle of NMS is to select the high score and delete the rest, so the $\lambda_i^3$ obtained by NMS is relatively large, that is, the eigenvalue corresponding to the feature point.

------

###### Summary

Now make comparisons among Harris 2D, Harris 3D with intensity, Harris 3D without intensity, Harris 6D, and ISS.

![image-20201026102334690](pics\16.png)

*Fig.8.2.5:   Comparisons among different feature*

Harris and ISS introduced above are the traditional method to calculate feature points, which performs better in the clean point cloud. Nevertheless, they are sensitive to the noise. Noise can make the covariance matrix satisfy the condition, so that generate some wrong judgements. Thus, deep learning is introduced.

##### Deep learning method

Although we hope to improve and optimize the extraction of feature points and other operations through deep learning, there are still difficulties in this kind of research because the definition of feature points is not clear, and there is no general data set. The point cloud itself is also characterized by sparseness, which will affect the operation of deep learning.

Next, we will introduce an algorithm for feature point manipulation using deep learning —— USIP.

###### USIP

USIP (Unsupervised Stable Interest Point) is an unsupervised learning algorithm. The red points in Fig.8.2.6 are the results of USIP:

![image-20200927143258553](pics\17.png)

*Fig.8.2.6:   USIP detection[^11]*

Fig.8.2.7 is the result of description and matching of feature points by USIP:

![image-20200927143618827](pics\18.png)

*Fig.8.2.7:   Description and matching[^11]*

USIP is an unsupervised learning algorithm, so which point is a feature point is determined by the network itself. The idea is as follows:

1. For a feature point, no matter from which angle it is viewed, it is still a feature point;
2. The concept of feature points is related to the relative area size. If we want to study a wheel, the pattern on the wheel may be the characteristic point. Nevertheless, if we want to study the entire car, then the pattern point on the wheel maybe not considered as a feature point since the relative area is too small.

The process of USIP is as following:

![image-20200927144531220](pics\19.png)

*Fig.8.2.8:   USIP[^11]*

***USIP algorithm structure:*** 

The first input is a three-dimensional point cloud $\mathbf X$. Then through Feature Proposal Network (it can be PointNet++ or So-Net, etc.) to get $M$ features $\mathbf Q$. Each feature point has one value to express uncertainty (>0), these uncertain values constitute $\Sigma$. Then for the point cloud $\mathbf X$ input at the beginning, do a random rotation and translation $T_l$ to get $\widetilde{\mathbf X}_l$. For $\widetilde{\mathbf X}_l$, perform FPN again to get the features $\widetilde{\mathbf Q}_l$ and $\widetilde{\mathbf \Sigma}_l$. Because a feature point should be a feature point no matter from which angle it is viewed, the difference of feature points we got twice should be a rotation and translation matrix $T_l$. Map $\widetilde{\mathbf Q}_l$ to the original space through $T_l$, and calculate the loss function with $\mathbf Q$ to make the loss function smallest.



#### 8-2-2：Description

For the description of feature points, there are traditional methods and deep learning approaches.

##### Traditional method

There are many traditional algorithms for feature point description. We have already touched PFH, FPFH and VFH in Chapter4. These are all descriptions of feature points, and we call them descriptors. This section will supplement SHOT related concepts.

###### SHOT：

SHOT (Signature of Histograms of OrienTations), to capture the information of the coordinates of each point, the steps are as follows:

***step1*:** Set the Local Reference Frame (LRF) for each point so that no matter how the object rotates, the LRF corresponds to the same. We can use PCA to obtain the principal components because the direction of these principal components will not change with the rotation and translation, which is the reaction of the structure of the object itself. A weighted covariance matrix (similar to ISS) is obtained according to the coordinates of the points and the neighborhood. After decomposing the matrix, the axes corresponding to the three principal components are obtained, but each axis will have two directions, positive and negative. We generally choose more points. As the final direction of the axis, after determining the two-axis directions, the third axis is obtained by the cross product of the other two axis directions.

***step2*:** After obtaining the LRF, divide the regional space of this LRF (the feature points and their neighborhoods, not the entire point cloud) into small volumes, as shown in the figure below, according to the latitude, longitude and radius direction:

<img src="pics\20.png" alt="image-20201026105404920" style="zoom:47%;" />

*Fig.8.2.9:   LRF and SHOT structure of division[^12]*

- Divide the longitude direction into eight blocks, Fig.8.2.9 is only  four blocks for clarity;
- Divide the latitude direction into two blocks;
- Divide the radius direction into two blocks, that is, a large ball  and a small purple ball;

In this way, we can obtain a total of $8× 2× 2 = 32$ small regions, and calculate a histogram in each small region. Then these small histograms are spliced together and standardized to obtain descriptors.

***Regarding the calculation of the histogram in each area:*** 

For a small area composed of feature points and their neighbors, calculate the normals of the feature points and their neighbors, denoted as $n$ and $n_i$, and store them in the histogram. The information is $cos\theta_i = n ·n_i$. The SHOT corresponding histogram in PCL is divided into 11 sub-intervals by default, so the obtained SHOT descriptor length is $11×32=352$, so the estimation class in PCL is *"pcl::SHOT352"*.

##### Deep learning method

There are four deep learning methods we will introduce: 3DMatch, The Perfect Match, PPFNet, PPF-FoldNet.

##### 3DMatch

The structure of 3DMAatch is:

![image-20200927170603579](pics\21.png)

*Fig.8.2.10:   3DMatch[^13]*

**(a)** Use the map reconstructed from the RGB-D dataset (which can be operated by SLAM) as the dataset, select several viewpoints: blue, yellow, and green, and then select several feature points (red points);

**(b)** is the three-angle view we selected in the RGB-D dataset as three patches. One of these patches is used as a benchmark (called the anchor). The other one (called positive) is the same scene, but different angles as the first one, and the last one is a view of a different scene (called negative);

**(c)** Establish VoxelGrid with the feature points in the three views as centres;

**(d)** Compare the feature point descriptions of the three views, and then define the loss function so that the descriptions of anchor and positive are consistent, and the descriptions of anchor and negative are inconsistent, to achieve the effect of training the network;

Choose Contrastive Loss as loss function.

First define label —— $y_{ij}$, which indicates whether the feature descriptions of $i$ and $j$ are consistent. If they are consistent (positive), then it is 1, otherwise (negative) is 0.

The Contrastive Loss is:
$$
\begin{array}{l}
L=\frac{1}{N} \sum_{n=1}^{N} y_{i j} d_{i j}^{2}+\left(1-y_{i j}\right) \max \left(\tau-d_{i j}, 0\right)^{2} \\
d_{i j}=\left\|f\left(x_{i}\right)-f\left(x_{j}\right)\right\|_{2}
\end{array}
$$
Among them, $x_i$ and $x_j$ represent different patches. $f(x)$ is to process the network to get the descriptor. Therefore, $d_{ij}$ is to measure the difference between the features of $x_i$ and $x_j$. If $x_i$ and $x_j$ are positive, then $y_{ij} = 1$ is substituted to minimize $d_{ij}$. If $x_i$ and $x_j$ are negative, define a threshold $\tau$. If $d_{ij}> \tau$, there is no need to deal with $d_{ij}$, otherwise $\tau-d_{ij}$ is minimized, that is, $d_{ij}$ is maximized.

**(e)** The application of this algorithm.

Fig.8.2.11 is the visualization of the effect of 3DMatch, which realizes clustering of patches that match as positive, indicating that these feature descriptors are distinguishable:

![image-20200927190701617](pics\22.png)

*Fig.8.2.11:   The effect of 3DMatch[^13]*

However, there are still many problems in 3DMatch:

- Cannot maintain stability for rotation and other operations well;
- The loss function is too strong, $d_{ij}$ does not have to be 0 or close to $\tau$, because the difference and similarity in the distance are relative.

Because of the problems of 3DMatch, we will introduce The Perfect Match next.

##### The Perfect Match

The Perfect Match improves the two problems of 3DMatch. The Perfect Match chooses to establish LRF regarding the effect of rotation.

###### The establishment of LRF

Similar to the establishment of LRF in SHOT, the covariance matrix of feature points and their neighbors is calculated (weighting is not required here), and the normal of the feature point is the z-axis. Similarly, the direction with more points is the z-axis direction. Project the neighbors of the feature point onto a plane perpendicular to the normal of the feature point. The weighted average of the projections of all the neighbors (the weight is bigger as the distance is closer) is the x-axis direction, and the y-axis direction is the cross product of z-axis direction and the x-axis direction.

For the problem that the loss function is too strong, The Perfect Match uses a weaker Triplet Loss.

###### Triplet Loss

The idea of Triplet Loss is that "Far" and "near" are relative, as long as the distance between anchor and positive is much greater than the distance between anchor and negative. It is not necessary that the former is infinitely close to 0 and the latter is close to $\tau$.

<img src="pics\23.png" alt="image-20200927201120873" style="zoom:60%;" />

*Fig.8.2.12:   Triplet Loss[^14]*
$$
L=\sum_{i}^{N}\left[\left\|f\left(x_{i}^{a}\right)-f\left(x_{i}^{p}\right)\right\|_{2}^{2}-\left\|f\left(x_{i}^{a}\right)-f\left(x_{i}^{n}\right)\right\|_{2}^{2}+\alpha\right]_{+}
$$
The process of minimizing $L$ is to minimize $\left\|f\left(x_{i}^{a}\right)-f\left(x_{i}^{p}\right)\right\|_{ 2}^{2}$, maximize $\left\|f\left(x_{i}^{a}\right)-f\left(x_{i}^{n}\right)\right\| _{2}^{2}$, $\alpha$ is the threshold (also the margin in Fig.8.1.13).

However, Triplet Loss also has its limitations:

- When the value of the loss function is close to 0, it cannot be used to monitor the network. It depends on how we look for negatives. If anchor and negative are randomly sampled in space, the probability of the two is very different;
- Slow network convergence;

As shown in Fig.8.2.13:

<img src="pics\24.png" alt="image-20200928080348685" style="zoom:36%;" />

*Fig.8.2.13:   The problems in Triplet Loss[^15]*

The hollow circle "a" represents anchor, and the hollow circle "p" represents positive. The distance between the two is the distance between the feature descriptors of the point. If we choose negative and fall into the green area, that is, Easy negatives, the distance between the negative and the anchor is too large, and the loss function is 0, which is too simple for the network. Nevertheless, if we choose negative, it falls into the red area, that is, Hard Negatives, which makes the loss function very large, cannot converge well or the convergence speed is prolonged. The best situation is that we choose negatives in the orange area, which is Semi-hard negatives.

Given this problem, ***Triplet Mining*** appeared. Find out which triples (anchor, positive, and negative) are good enough so that our loss function will neither be 0 nor cannot converge well. The triples are generally selected in the Semi-hard or Hard area. Try not to choose the ones in the Easy area. Otherwise, the network will not be well supervised.

##### PPFNet

PPFNet (Point Pair Feature Network) is a feature description network based on PointNet.

![image-20200928144939775](pics\25.png)

*Fig.8.2.14:   PPFNet[^16]*

Different from 3DMatch and The Perfect Match, the input of PPFNet is not every patch, but a frame of the point cloud at the same time. After inputting the frame, many patches can be obtained through sampling techniques, and these patches will pass through a shared PointNet to obtain Local Feature. Perform Max Pooling on these obtained Local features to obtain Global Feature. Then the Local Feature and Global Feature are spliced together to get the information of each patch and the overall information of the frame. Then enter MLP to get the final feature descriptors.

###### Innovation 1: PPFNet input information

In the patch input by PPFNet, not only the coordinates of the point cloud but also other information of the point cloud, such as normal, PFH... Because the richer the input of deep learning, the better the result will be accordingly. The composition of this information is introduced below, as shown below:

<img src="pics\26.png" alt="image-20200928150615818" style="zoom:66%;" />

*Fig.8.2.15:   PPFNet innovation[^16]*

Among them, $x_r$ is the feature point of this patch, $n_r$ is the normal of the feature point, $x_i$ is the neighbor, and $n_i$ is the normal of the neighbor. Connect each neighbor to the feature point (similar to SPFH idea, see in Chapter4). Each point pair composed of a neighbor and a feature point contains the following PPF four-tuple information:
$$
\begin{array}{l}
\psi_{12}=\left(\|\mathbf{d}\|_{2}, \angle\left(\mathbf{n}_{1}, \mathbf{d}\right), \angle\left(\mathbf{n}_{2}, \mathbf{d}\right), \angle\left(\mathbf{n}_{1}, \mathbf{n}_{2}\right)\right) \\
\angle\left(\mathbf{v}_{1}, \mathbf{v}_{2}\right)=\operatorname{atan} 2\left(\left\|\mathbf{v}_{1} \times \mathbf{v}_{2}\right\|, \mathbf{v}_{1} \cdot \mathbf{v}_{2}\right)
\end{array}
$$
Among them, $||\mathbf d||_2$ is the distance between each neighbor and the feature point. The remaining three angles are the angles between the respective normals of the two points and the line connecting them, and the angle between two normals. Combine the coordinates and normals of the feature points and neighbors, as well as the four-tuple combination we just got, and combine them in a matrix to get the $\mathbf F_r$ on the right in Fig.8.2.15. Since the coordinates and normals of the points are all three-dimensional, and the quadruple calculated for each neighbor and feature point is four-dimensional, there must be missing places in $\mathbf F_r$, just fill in zeros. Once written in matrix form, it can be processed by PointNet.

###### Innovation 2: The process of training

The 3DMatch introduced earlier is a two-tuple (Anchor and Positive or Anchor and Negative) when training the loss function. The Perfect Match uses a triple (Anchor, Positive, and Negative), which are all a patch. The input of PPFNet is a frame, so the training of the loss function is also different.

*N-tuple Loss*

3DMatch and The Perfect Match only consider the information in one patch, while PPFNet considers the information of $n$ (hyper-parameters, artificially specified) patches in a frame. The comparison of the three ideas in the following figure, the red is pushed away, indicating different; Green is closer, indicating similarity. The problem in (a) and (b) is that it does not bring all the similar shapes closer, and (c) can do:

![image-20200928153121898](pics\27.png)

*Fig.8.2.16:   Comparisons of the establishment of loss function[^16]*

Steps:

**step1**: Choose $n$ patches for each frame. There are two frames. Record the location of the feature point in the feature space $x_i$ in frame1, and the location of the feature point in the feature space $y_i$ in frame2. During training, the rotation and translation matrix $\mathbf T$ of frame1 and frame2 is known.

**step2**: Determine the corresponding positive and negative, the one with the same distance in the feature space is positive, otherwise negative. The judgment formula is:
$$
m_{i j}=\mathbb{1}\left(\left\|\mathbf{x}_{i}-\mathbf{T} \mathbf{y}_{j}\right\|_{2}<\tau\right)
$$
Among them, $\mathbb{1}$ is an indicator function. $m_{ij}$ composes a matrix $\mathbf M$ of the similarity relationship of feature points.

**step3**: Calculate the feature distance matrix $\mathbf D$, where $f(·)$ is PPFNet, and the calculation formula for each element of the matrix is as follows:
$$
d_{i j}=\left\|f\left(\mathbf{x}_{i}\right)-f\left(\mathbf{y}_{j}\right)\right\|_{2}
$$
**step4**: get N-tuple Loss:
$$
L=\sum^{*}\left(\frac{\mathbf{M} \circ \mathbf{D}}{\|\mathbf{M}\|_{2}^{2}}+\alpha \frac{\max (\theta-(1-\mathbf{M}) \circ \mathbf{D}, 0)}{N^{2}-\|\mathbf{M}\|_{2}^{2}}\right)
$$

> We will not expand the mathematical derivation here.

The following figure is a schematic diagram of the PPFNet training process:

![image-20200928155739736](pics\28.png)



*Fig.8.2.17:   The process of training PPFNet[^16]*

Among them, Ground Truth Poses is the $\mathbf T$ in step 1, the Correspondence Matrix obtained by Distance Matrix + Binarize (indicating function $\mathbb 1$) is the matrix $\mathbf M$ in step 2, and the Feature Space Distance Matrix is the matrix $\mathbf D$ in step 3. 

The figure below is a comparison chart of several different loss functions. Pair means Contrastive Loss, Triplet means Triplet Loss, and N-tuple means N-tuple Loss. Purple is negative and red is positive. It shows that N-tuple Loss distinguishes between positive and negative better: the positive distance is close to 0, and the negative distance is larger.

![image-20200928160310489](pics\29.png)

*Fig.8.2.18:   Comparison of several loss functions[^16]*

##### PPF-FoldNet

The PPF-FoldNet framework is shown in Fig.8.2.19, the input is still a patch:

![image-20200928161339539](pics\31.png)

*Fig.8.2.19:   PPF-FoldNet[^17]*

Input a patch, and then express the patch through PPF (there are no coordinates and normals of points, only PPF quadruples, so it is 4D). The feature vector —— codeword is obtained through encoder, and the reconstructed patch is obtained through decoder. The original patch —— $\mathbf F$ and the reconstructed patch —— $\hat{\mathbf F}$ should be the same. The loss function is Chamfer Loss:
$$
d(\mathbf{F}, \hat{\mathbf{F}})=\max \left\{\frac{1}{|\mathbf{F}|} \sum_{\mathbf{f} \in \mathbf{F}} \min _{\mathbf{f} \in \hat{\mathbf{F}}}\|\mathbf{f}-\hat{\mathbf{f}}\|_{2}, \frac{1}{|\hat{\mathbf{F}}|} \sum_{\mathbf{f} \in \hat{\mathbf{F}}} \min _{\mathbf{f} \in \mathbf{F}}\|\mathbf{f}-\hat{\mathbf{f}}\|_{2}\right\}
$$
Fig.8.2.20 is a comparison between the original feature visualization after T training, and it can be found that after 70 rounds of training, the reconstructed feature visualization is very close to Original PPF:

![image-20200928164336608](pics\30.png)

*Fig.8.2.20:   Training rounds and visualization[^17]*



#### 8-2-3：Matching

Feature description matching is the registration between point clouds. The ultimate goal is to find a rigid transformation matrix combining a rotation matrix and a translation vector. We have introduced the ICP algorithm for registration in Chapter2 and 4. The registration and the processing of feature points are summarized here to promote a further understanding of the registration.

> NB: All currently encountered are rigid matching. Non-rigidity will be relatively troublesome, so no expansion is done here.

##### RANSAC Registration

Common registration algorithms, such as ICP and NDT (Normal Distribution Transformation, which is a traditional registration algorithm different from ICP, and will not be explained in detail here), all need to have a good initial solution. Otherwise the registration effect It may not be right. However, the registration method based on feature extraction and description can solve this problem. Of course, the result of the feature-based registration method is not accurate, so we often combine the two registrations. First, obtain a better initial solution through feature-based registration, and then use ICP or NDT to get a more accurate Registration result.

The feature-based registration introduced here is RANSAC (Random Sample Consensus) registration. Proceed as follows:

![image-20200928195454679](pics\32.png)

*Fig.8.2.21:   Registration[^11]*

**step1**: Perform feature extraction and description on the source point cloud and target point cloud to obtain feature descriptors;

**step2**: Estimating the correspondence relationship (according to the distance between the feature descriptors, the distance here is not Euclidean distance, but the distance in the feature space). How to establish this correspondence relationship?

First, enter the feature description of the target point cloud and the source point cloud. Secondly, find the correspondence in the following ways:

① For each point in the target point cloud, find the point with the most similar feature in the source point cloud (note that it is not the Euclidean distance, but the similarity in the feature vector space);

② For each point in the source point cloud, find the point with the most similar features in the target point cloud (note the same as above);

③ Combines ① and ②, search in both directions, and the one that satisfies one of the two can be regarded as the corresponding point pair (note the same as above);

④ Combines ① and ② to search in both directions. But only if both directions correspond to each other, it is recorded as a corresponding point. For example, according to the point A in the target point cloud, it is found that the point B in the source point cloud has the most similar features. Vice versa, the corresponding point found for the point B in the source point cloud is also the point A in the target point cloud, then A and B is the corresponding point pair (note the same as above);

**step3**: Operate RANSAC iteration. Consider 3 point pairs each time, estimate the rotation matrix and translation vector through Procrustes Transformation, and then determine the inliers according to the Euclidean distance of each point from the corresponding point after the transformation;

**step4**: Select the rotation matrix and translation vector with the most inliers calculated.

------

The following summarizes a general point cloud registration method:

- Data preprocessing: downsampling, noise reduction, filtering;
- To find the initial rotation matrix and translation vector. The result obtained by prior knowledge or feature description matching can be used as the initial solution;
- After the initial solution is obtained, methods such as ICP and NDT are used to optimize the rotation matrix and translation vector further.

> The above is an introduction to the relevant knowledge of the feature of this chapter. Part of the code is open source, you can try if you are interested.
>
> This chapter refers to the 3D point cloud processing course of Shenlanxueyuan https://www.shenlanxueyuan.com/course/262



### Reference

[^1]: http://cs.brown.edu/courses/cs143/2013/results/proj2/taparson/
[^2]: http://www.magicandlove.com/blog/2014/03/13/opencv-features2d-in-processing/#more-1252
[^3]: https://www.programmersought.com/article/86603333968/
[^4]: Computer Vision, Raquel Urtasun

[^5]: https://webdiis.unizar.es/~raulmur/orbslam/。
[^6]: CSE486, Penn State, Robert Collins
[^7]: 16-385 Computer Vision, CMU, Kris Kitani

[^8]: https://prs.igp.ethz.ch/research/completed_projects/automatic_registration_of_point_clouds.html
[^9]: http://www.fubin.org/research/Human_Pose_Estimation/Human_Pose_Estimation.html
[^10]: https://youtu.be/xMgoypPBEgw
[^11]: Li, J., & Lee, G. H. (2019). Usip: Unsupervised stable interest point detection from 3d point clouds. In *Proceedings of the IEEE International Conference on Computer Vision* (pp. 361-370).
[^12]: Tombari, F., Salti, S., & Di Stefano, L. (2010, September). Unique signatures of histograms for local surface description. In *European conference on computer vision* (pp. 356-369). Springer, Berlin, Heidelberg.
[^13]: Zeng, A., Song, S., Nießner, M., Fisher, M., Xiao, J., & Funkhouser, T. (2017). 3dmatch: Learning local geometric descriptors from rgb-d reconstructions. In *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition* (pp. 1802-1811).
[^14]: Schroff, F., Kalenichenko, D., & Philbin, J. (2015). Facenet: A unified embedding for face recognition and clustering. In *Proceedings of the IEEE conference on computer vision and pattern recognition* (pp. 815-823).
[^15]: https://omoindrot.github.io/triplet-loss
[^16]: Deng, H., Birdal, T., & Ilic, S. (2018). Ppfnet: Global context aware local features for robust 3d point matching. In *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition* (pp. 195-205).
[^17]: Deng, H., Birdal, T., & Ilic, S. (2018). Ppf-foldnet: Unsupervised learning of rotation invariant 3d local descriptors. In *Proceedings of the European Conference on Computer Vision (ECCV)* (pp. 602-618).

