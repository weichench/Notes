## Tracking

+ 跟踪局部地图
  + 根据covisibility-graph，插入与当前帧有共视地图点的关键帧，spananing tree 
  + 更新局部地图点，在局部地图上优化BA
+ 

_________________

## local mapping

+ local BA
  + 和当前关键帧相连的关键帧及map points做局部BA优化
  + 





_________

## 一些疑问

+ 帧间跟踪时，比如跟踪上一关键帧，如何通过BOW实现特征点的快速匹配？
+ g2o的使用与ceers的差异？