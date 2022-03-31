# Package - Planning


## Function

Due to the sensor capbility, the planning task can be divided into three parts in terms of distance between victims and UAV:
1. distance > 60m  
   In this range, UAV can't reveived any signals from the victim, so we predefine some observation points which make up a basic trajectory. Following this trajectory, UAV can reach the places where it can sense every victims at least once. 

2. 22m < distance <= 60m  
   In this range, UAV can receive signals with relative higher noise, which makes the signals unusable for calculation of the  victim's location. In this period, UAV chooses the victim with greatest signal strength (which means shortest distance). Then it moves forwards, backwards, leftwards and rightwards to collect the signal strength regarding different own positions. Using these data, UAV will derive a direction which makes the signal strength increase most rapidly, and fly along this direction. During this time, UAV will keep checking if the signal strength is still increasing. Once the answer is no, it will stop and move forwards, backwards, leftwards and rightwards again to get a new direction. This loop will last until UAV stays within 22m of the victims (due to the accuracy of collected data, we actually stop at 20m).

3. distance <= 22m  
   In this range, UAV can collect the signals with lower noise, and it will fly to 6 points with certain displacements with current position. We will use Four-Point locolization method to calculate the concrete position of targeted victim. After finishing this, UAV will continue to find other victims or move to next observation point.

For more information about the algorithm, please refer to our [report](https://gitlab.lrz.de/ge23ged/autonomous-systems-2021-group-terminus/-/blob/main/project/Report/Terminus_Report.pdf).


