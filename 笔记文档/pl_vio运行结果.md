

## 原始结果  ，去头

       max	0.095584
      mean	0.032809
    median	0.029472
       min	0.005095
      rmse	0.038790
       sse	0.586808
       std	0.020693


## 对线特征的匹配做了改进后的结果

       max	0.103396
      mean	0.037647
    median	0.039552
       min	0.005022
      rmse	0.042042
       sse	0.705246
       std	0.018714


## 通过线的中点删选误匹配

       max	0.098347
      mean	0.037588
    median	0.037238
       min	0.004514
      rmse	0.042580
       sse	0.723403
       std	0.020004
## 增加双向匹配，解决同一帧出现相同ID的情况，提前筛选出误匹配， 并且，在前后匹配中，前后匹配不一致所涉及的两个id都进行了删除

       max	0.112774
      mean	0.041730
    median	0.044553
       min	0.004437
      rmse	0.047401
       sse	0.896479
       std	0.022482


## 只删除curframe_->lineID_reverse_front中出现的ID

       max	0.101357
      mean	0.034984
    median	0.037611
       min	0.006176
      rmse	0.039421
       sse	0.620052
       std	0.018170


##  只删除curframe_->lineID中出现的不一致的ID

       max	0.095577
      mean	0.033808
    median	0.032893
       min	0.004471
      rmse	0.038436
       sse	0.589458
       std	0.018285
## 对误匹配的ID不做改变

       max	0.102976
      mean	0.034308
    median	0.036638
       min	0.005372
      rmse	0.038731
       sse	0.598523
       std	0.017972

##  用中点距离与端点距离共同筛选误匹配, 只删除curframe_->lineID中出现的不一致的ID

       max	0.088880
      mean	0.032853
    median	0.030900
       min	0.003064
      rmse	0.037601
       sse	0.564113
       std	0.018290
## 添加了线的合并，只要线特征进行了三角化，且三角化所在的帧小于8，都会考虑去合并

       max	0.087596
      mean	0.029687
    median	0.029679
       min	0.001969
      rmse	0.034113
       sse	0.464322
       std	0.016804



## 修改了相机系下直线与原点的距离，直线外点的剔除仅通过最小重投影误差与直线到原点的距离

    max	0.083035
      mean	0.029528
    median	0.028819
       min	0.002168
      rmse	0.033647
       sse	0.451724
       std	0.016131
## 将只有通过优化的线才去做合并

       max	0.089865
      mean	0.032698
    median	0.031504
       min	0.001551
      rmse	0.037057
       sse	0.547921
       std	0.017438


##  只在onlyStructuralLineOpt中添加ProjectionStructuralLine_Oneframe

     max	0.088202
      mean	0.032945
    median	0.031662
       min	0.002550
      rmse	0.037726
       sse	0.567866
       std	0.018380


## 继续在联合优化函数中添加ProjectionStructuralLine_Oneframe  极差！

       max	0.657619
      mean	0.224379
    median	0.180291
       min	0.032154
      rmse	0.266784
       sse	28.398376
       std	0.144319


## 边缘化的时候也添加ProjectionStructuralLine_Oneframe  更差！！！

    max	0.855755
      mean	0.316907
    median	0.338127
       min	0.025710
      rmse	0.368632
       sse	54.220030
       std	0.188308















## 60s

Compared 1189 absolute pose pairs.
Calculating APE for translation part pose relation...
--------------------------------------------------------------------------------
APE w.r.t. translation part (m)
(with Sim(3) Umeyama alignment)

       max	0.220872
      mean	0.078454
    median	0.076258
       min	0.020363
      rmse	0.088878
       sse	9.392367
       std	0.041765
______

# Rws

## onlyStructuralLineOpt 固定位姿

Cost:
Initial                          2.594357e+02
Final                            1.212689e+02
Change                           1.381669e+02

Minimizer iterations                        9
Successful steps                            9
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0005

  Residual evaluation                  0.0025
  Jacobian evaluation                  0.0093
  Linear solver                        0.0036
Minimizer                              0.0164

Postprocessor                          0.0000
Total                                  0.0169

## 前一步固定位姿，联合优化的结果， 

Cost:
Initial                          6.555163e+06
Final                            1.701531e+03
Change                           6.553461e+06

Minimizer iterations                        9
Successful steps                            9
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0056

  Residual evaluation                  0.0861
  Jacobian evaluation                  0.1468
  Linear solver                        0.0813
Minimizer                              0.3273

Postprocessor                          0.0003
Total                                  0.3332



## 直接一步联合优化的结果

Cost:
Initial                          6.556008e+06
Final                            2.186018e+03
Change                           6.553822e+06

Minimizer iterations                        9
Successful steps                            9
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0041

  Residual evaluation                  0.0403
  Jacobian evaluation                  0.0652
  Linear solver                        0.0826
Minimizer                              0.2045

Postprocessor                          0.0003
Total                                  0.2089





##　不固定位姿

Cost:
Initial                          2.594357e+02
Final                            1.168076e+02
Change                           1.426282e+02

Minimizer iterations                        9
Successful steps                            6
Unsuccessful steps                          3

Time (in seconds):
Preprocessor                           0.0008

  Residual evaluation                  0.0026
  Jacobian evaluation                  0.0084
  Linear solver                        0.0134
Minimizer                              0.0257

Postprocessor                          0.0000
Total                                  0.0265

## 前一步不固定位姿，后一步的联合优化结果，Rws变化很大

Initial                          9.114700e+06
Final                            1.676733e+03
Change                           9.113023e+06

Minimizer iterations                        9
Successful steps                            9
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0045

  Residual evaluation                  0.0843
  Jacobian evaluation                  0.1369
  Linear solver                        0.0724
Minimizer                              0.3059

Postprocessor                          0.0002
Total                                  0.3106

## 前一步不固定位姿，不剔除外点————很明显，不固定位姿，反倒让误差变大

Initial                          9.116959e+06
Final                            2.186019e+03
Change                           9.114773e+06

Minimizer iterations                        9
Successful steps                            9
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0056

  Residual evaluation                  0.0395
  Jacobian evaluation                  0.0762
  Linear solver                        0.0831
Minimizer                              0.2154

Postprocessor                          0.0002
Total                                  0.2213

## 前一步固定位姿，但不剔除外点

Initial                          6.555653e+06
Final                            2.186012e+03
Change                           6.553467e+06

Minimizer iterations                        9
Successful steps                            9
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                           0.0050

  Residual evaluation                  0.0483
  Jacobian evaluation                  0.0759
  Linear solver                        0.0847
Minimizer                              0.2248

Postprocessor                          0.0003
Total                                  0.2301



## 后一步联合优化时删除的线 11 

[ WARN] [1587988576.303011795]: ID:112, deleted  because  distance
[ WARN] [1587988576.303166765]: ID:122, deleted  because  distance
[ WARN] [1587988576.303240095]: ID:67,  reproj_err :0.012318, erased
[ WARN] [1587988576.303317336]: ID:37,  reproj_err :0.011219, erased
[ WARN] [1587988576.303515327]: ID:110,  reproj_err :0.053600, erased
[ WARN] [1587988576.303573014]: ID:136, deleted  because  distance
[ WARN] [1587988576.303635588]: ID:142, deleted  because  distance
[ WARN] [1587988576.303696697]: ID:165, deleted  because  distance
[ WARN] [1587988576.303753894]: ID:166, deleted  because  distance
[ WARN] [1587988576.303814025]: ID:230,  reproj_err :0.063950, erased
[ WARN] [1587988576.303871711]: ID:233, deleted  because  distance



## 前一步固定位姿，优化后删除的外点8+4

[ WARN] [1587989086.626574881]: ID:122, deleted  because  distance
[ WARN] [1587989086.626678522]: ID:67,  reproj_err :0.012461, erased
[ WARN] [1587989086.626758208]: ID:37,  reproj_err :0.013187, erased
[ WARN] [1587989086.626812473]: ID:51,  reproj_err :0.008142, erased
[ WARN] [1587989086.626872604]: ID:110,  reproj_err :0.053950, erased
[ WARN] [1587989086.626918558]: ID:136, deleted  because  distance
[ WARN] [1587989086.626983089]: ID:166, deleted  because  distance
[ WARN] [1587989086.627044198]: ID:230,  reproj_err :0.063872, erased

## 随后的联合优化删除的线

[ WARN] [1587989170.461833469]: ID:112, deleted  because  distance
[ WARN] [1587989170.461911199]: ID:142, deleted  because  distance
[ WARN] [1587989170.461976708]: ID:165, deleted  because  distance
[ WARN] [1587989170.462036351]: ID:233, deleted  because  distance



## 直接一步联合优化删除的线（11）

[ WARN] [1587989521.968242945]: ID:112, deleted  because  distance
[ WARN] [1587989521.968299158]: ID:122, deleted  because  distance
[ WARN] [1587989521.968352927]: ID:67,  reproj_err :0.012421, erased
[ WARN] [1587989521.968424293]: ID:37,  reproj_err :0.006934, erased
[ WARN] [1587989521.968490771]: ID:110,  reproj_err :0.053301, erased
[ WARN] [1587989521.968541607]: ID:136, deleted  because  distance
[ WARN] [1587989521.968589021]: ID:142, deleted  because  distance
[ WARN] [1587989521.968651588]: ID:165, deleted  because  distance
[ WARN] [1587989521.968700958]: ID:166, deleted  because  distance
[ WARN] [1587989521.968758148]: ID:230,  reproj_err :0.063869, erased
[ WARN] [1587989521.968809473]: ID:233, deleted  because  distance