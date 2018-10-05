# path
path search

问题定义
定义模型:
起始位置（0,0）,目标位置(4,6);障碍物设置成圆，其中圆的中心点坐标及半径分别设置为(1.5,4.5,1.5),(4.0,3.0,1.5),(1.2,1.5,0.8);成本函数为MyCost;处理点的个数为3,变量的搜索范围（-10,10）。

PSO 参数
Pso参数包括最大迭代次数500，群体规模10，权重最小值0.2，权重最大值0.98。惯性权重初始值为权重最大值，个体学习系数1.5，全体学习系数1.5，x方向和y方向的最大与最小速度。

SA参数
模拟退火参数包括种群迭代次数500，子种群迭代次数20，初始温度0.1，突变率0.5。

初始化
1）创建空粒子结构，初始化全局最佳INF（无穷大），创建粒子矩阵。在此基础上构建源到目标的直线，初始化速度样本，更新个体和全局最佳成本，更新矩阵。
2）设置初始温度T0（模拟退火的初始温度）

算法描述
此部分涉及PSO算法和SA算法优化。
PSO算法，涉及速度和位置的更新。其中速度的更新包括三个部分，第一部分反映粒子的惯性，第二部分反映自身的影响，第三部分反映整个粒子群的影响。在检查速度边界值后更新位置，接着利用CostFunction 函数计算成本，在此基础上个体最佳和全局最佳以及变量自身的权重。
核心代码：
%%Xpart：
  particle(i).Velocity.x=w*particle(i).Velocity.x...
                +c1*rand(VarSize).*(particle(i).Best.Position.x-particle(i).Position.x)...
                +c2*rand(VarSize).*(GlobalBest.Position.x-particle(i).Position.x);
其中权重的优化：w=Wmin+(Wmax-Wmin)*exp((MaxIt-it)/(MaxIt-1)*q)/exp(q);

SA算法：
若J( Y(i+1) )>= J( Y(i) )  (即移动后得到更优解)，则总是接受该移动
         若J( Y(i+1) )< J( Y(i) )  (即移动后的解比当前解要差)，则以一定的概率接受移动，而且这个概率随着时间推移逐渐降低（逐渐降低才能趋向稳定）
核心代码：
  for i=1:nPop
            
            if newpop(i).Cost<=particle(i).Cost
                particle(i)=newpop(i);
                
            else
                DELTA=(newpop(i).Cost-particle(i).Cost)/particle(i).Cost;
                P=exp(-DELTA/T);
                if rand<=P
                    particle(i)=newpop(i);
                end
            end
最后显示迭代信息和图示Model解决方案。

  

实验结果分析
根据最佳成本，绘制模型的迭代图。根据所绘制的图像，我们可以清晰地看到在PSO算法中达到局部最优值9.2附近，随着SA算法迭代的进行，跳出这个局部最优解，最终程序收l敛到7.6附近。   

