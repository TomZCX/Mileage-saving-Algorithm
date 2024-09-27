% 节约里程法计算VRP车辆和路径方案
% 主程序--具有绘图功能
function vrp02_cw()
    [cap,xyPos,dmdQty]=initDataE22_K4();
    dists=createDist(xyPos)
    savings=savingDist(dists)
    [route1,singleWeight]=CWPSaving(savings,cap,dmdQty(:,2))
    distPsavel=getSumDist(route1,dists)
    drawRoute(route1,xyPos);
    
end

% Step1,2
% 数据的初始化
function[capacity,xyPos,demand]=initDataE22_K4()
    capacity=6000
    xyPos=[1 145 215; 2 151 264; 3 159 261; 4 130 254; 5 128 252; 6 163 247; 7 146 246; 8 161 242; 
        9 142 239; 10 163 236; 11 148 232; 12 128 231; 13 156 217; 14 129 214; 15 146 208; 16 164 208; 
        17 141 206; 18 147 193; 19 164 193; 20 129 189; 21 155 185; 22 139 182];
    demand=[1 0; 2 1100; 3 700; 4 800; 5 1400; 6 2100; 7 400; 8 800; 9 100; 10 500; 11 600; 12 1200;
        13 1300; 14 1300; 15 300; 16 900; 17 2100; 18 1000; 19 900; 20 2500; 21 1800; 22 700];
end


% Step3
%根据点的xy坐标，求各点之间的距离矩阵
function nowDist=createDist(xyPos)
    newXY=xyPos(:,2:3)';
    nowDist=dist(newXY);
end


% Step4
%节约里程数计算,并按照节约里程数降序排序
function savingArray=savingDist(pDist)
    siteQty=size(pDist,1);
    savingArray=zeros(nchoosek(siteQty-1,2),3);     % 生成一个数组对应的数据框架
    row=1;
    for i=2:siteQty-1
        for j=i+1:siteQty
            point1=i;
            point2=j;
            saving=pDist(1,i)+pDist(1,j)-pDist(i,j);
            savingArray(row,1:3)=[point1,point2,saving];
            row=row+1;
        end
    end
    savingArray=sortrows(savingArray,-3);       % 将得到的节约里程数表按第3列降序排序
end


% Step5
% 并行方式进行节点合并操作，形成最后的路径方案
function [route,singleWeight]=CWPSaving(sortSaveVal, truckCap, xyDmd)
  [row,~]=max(size(xyDmd));
  if row>2
     customerQty=size(xyDmd,1)-1;
     route=zeros(customerQty,customerQty+2);
     finishNodeQty=0;setRouteQty=0;finishNodes=zeros(1,customerQty+1);
     finishNodes(1)=1;
     routeWeight=zeros(customerQty,1);

     saveRows=size(sortSaveVal,1);
     for i=1:saveRows
        nodeA=sortSaveVal(i,1);nodeB=sortSaveVal (i,2);
        [rowA,colA]=find(route==nodeA);
        [rowB,colB]=find(route==nodeB);
        isFindA=sum(rowA)+sum(colA);
        isFindB=sum (rowB)+sum(colB);
        if isFindA+isFindB==0
           sumDmd=xyDmd (nodeA)+xyDmd (nodeB);
           midIdx=find(route(:,1)==0);midIdx2=sum(midIdx);
           if midIdx2>0
              thisRow=midIdx(1);
              if sumDmd<=truckCap
                 route(thisRow, 1)=1;route(thisRow, 2)=nodeA;route(thisRow, 3)=nodeB;
                 finishNodeQty=finishNodeQty+2;setRouteQty=setRouteQty+1;
                 finishNodes (nodeA)=finishNodes (nodeA)+1;finishNodes (nodeB)=finishNodes (nodeB)+1;
                 routeWeight(thisRow)=routeWeight(thisRow)+sumDmd; 
              end
           else
               disp('error l in jVRP.clarkeWrightPs');
           end
         elseif isFindA+isFindB>0
           if isFindA*isFindB==0
             if isFindA>0
               newNode=nodeB;addRow=rowA(1); addCol=colA(1);
             else
               newNode=nodeA;addRow=rowB(1);addCol=colB(1);
             end
             midWeight=routeWeight(addRow)+xyDmd (newNode);

             if midWeight<=truckCap
               midRouteLength=max(find(route(addRow,:)>1));
               if addCol==2
                 route(addRow, 3:midRouteLength+1)=route (addRow, 2:midRouteLength);
                 route(addRow,2)=newNode;routeWeight(addRow)=midWeight;
                 finishNodeQty=finishNodeQty+1;finishNodes (newNode)=finishNodes (newNode)+1;
               elseif addCol==midRouteLength
                 route(addRow, midRouteLength+1)=newNode;routeWeight (addRow)=midWeight;
                 finishNodeQty=finishNodeQty+1;finishNodes (newNode)=finishNodes (newNode)+1;
               end
             end
           else
             midWeight=routeWeight(rowA(1))+routeWeight(rowB(1));
             if midWeight<=truckCap &&rowA(1)~=rowB(1)
               midRouteLthA=max(find(route(rowA(1),:)>1));
               midRouteLthB=max(find(route(rowB(1),:)>1));
               if colA(1)==2 &&colB(1)==2
                 exchangeQty=midRouteLthB-l;
                 exchangeRoute=route(rowB(1),2:midRouteLthB);
                 exchgRoute=fliplr(exchangeRoute);
                 route (rowA (1), exchangeQty+2:exchangeQty+midRouteLthA)=route(rowA(1),2:midRouteLthA); 
                 route(rowA(1),2:exchangeQty+1)=exchgRoute;
                 route(rowB(1),:)=0;
                 routeWeight(rowA(1))=midWeight;
                 routeWeight(rowB(1))=0;setRouteQty=setRouteQty-1;
               elseif colA(1)==2 &&co1B(1)==midRouteLthB
                 exchangeQty=midRouteLthA-l;
                 exchangeRoute=route(rowA(l),2:midRouteLthA);
                 route (rowB (1), midRoutelthB+l:midRoutelthB+exchangeQty)=exchangeRoute;
                 route(rowA(1),:)=0;
                 routeWeight (rowB(1))=midWeight;
                 routeWeight(rowA(1))=0;setRouteQty=setRouteQty-1;
               elseif colB(1)==2 &&colA(1)==midRouteLthA
                 exchangeQty=midRoutelthB-l;
                 exchangeRoute=route(rowB(1),2:midRouteLthB);
                 route(rowA(1),midRoutelthA+l:exchangeQty+midRouteLthA)=exchangeRoute;
                 route(rowB(1),:)=0;
                 routeWeight(rowA(1))=midWeight;
                 routeWeight(rowB(1))=0;setRouteQty=setRouteQty-l;
                elseif colA(1)==midRoutelthA &&co1B(1)==midRouteLthB
                  exchangeQty=midRouteLthB-l;
                  exchangeRoute=route(rowB(1),2:midRouteLthB);
                  exchgRoute=fliplr(exchangeRoute);
                  route(rowA(1), midRouteLthA+l:midRoutelthA+exchangeQty)=exchgRoute;
                  route(rowB(1),:)=0;
                  routeWeight (rowA(1))=midWeight;
                  routeWeight(rowB(1))=0;setRouteQty=setRouteQty-1;
                end
              end
            end
        end
     end
        route=sortrows(route,-1);
        midIdx=max(find(route(:,1)>0));
        if midIdx~=setRouteQty
          disp('error 2 in jVRP. clarkeWrightPS');
        end
        if finishNodeQty<customerQty
          needAssignNodes=find(finishNodes==0);
          isHas=sum(needAssignNodes);
          if isHas==0
           disp('error 3 in jVRP.clarkeWrightPs');
          end
          unFinishQty=size(needAssignNodes, 2);
          for  j=l:unFinishQty
            route(setRouteQty+j,1)=l;
            route(setRouteQty+j,2)=needAssignNodes(j);
          end
        end
        midIdx=find(route(:,1)==0);
        route(midIdx, :)=[];     [row, col]=size(route);
        for i=1:row
           midRouteLth=max (find (route(i, :)>1));
           route (i, midRouteLth+1)=1;
        end
        maxNum=0;
        for i=1:row
          xx=find(route(i,:)==1);
          midV=max(xx);
          if midV>maxNum
            maxNum=midV;
          end
        end
        if maxNum<col
          route(:, maxNum+1:col)=[];
        end
  end
      idx=routeWeight==0;
      routeWeight (idx)=[];
      singleWeight=routeWeight;
    end

   
% Step6    
% 生成路径方案的总里程
function sumDist=getSumDist(route,dists)
   [rows,cols]=size(route);
   sumDist=0
   for i=1:rows
       for j=2:cols
           if route(i,j)>0
              sumDist=sumDist+dists(route(i,j-1),route(i,j));
           end
       end
   end
end


%  绘制图形
function drawRoute(route, xyPos)
 [rows, cols]=size(route);
 colors=rand(rows,3)
 for i=1:rows
     nowRoute=route(i,:);
     goodCols=0;
     for j=1:cols
         if nowRoute(j)>0
             goodCols=j;
         else
             break;
         end
     end
     x=zeros(1,goodCols);
     y=zeros(1,goodCols);
     for j=1:cols;
         if nowRoute(j)>0
            nowPld=nowRoute(j);
            x(j)=xyPos(nowPld, 2);
            y(j)=xyPos(nowPld, 3);
         else
             break
         end
     end

 line(x,y,'color',colors(i,:),'Marker','o','MarkerSize',5,'MarkerEdgeColor','b');

 end

 % 节点坐标处绘制节点编号
 for i=1:size(xyPos,1)
     x=xyPos(i,2)+0.5;y=xyPos(i,3);
     text(x,y,int2str(i));
 end
end


