L = 5;
X = 4.3301;
Y = 7.5;
theta1 = [30;0];
theta2 = [30;0];
population = 100;
childStartGenome =floor(rand(100,4)*10);
errorCompare = [0 0];
for i = 1:population
childGenome(i,1) = 0;
childGenome(i,13) = 0;
childGenome(i,25) = 0;
childGenome(i,37) = 0;
childGenome(i,2:12) = de2bi(childStartGenome(i,1),11);
childGenome(i,14:24) = de2bi(childStartGenome(i,2),11);
childGenome(i,26:36) = de2bi(childStartGenome(i,3),11);
childGenome(i,38:48) = de2bi(childStartGenome(i,4),11);
end
generation = 0;
flag = 0;
while(flag == 0)
    count = 0;
    fitnessValue = zeros(population,3);
    for arm = 1:population
        Theta1Pred(1,1) = 30;
        Theta2Pred(1,1) = 30;
        theta1 = [30;0;0];
        theta2 = [30;0;0];
        ReqTheta1(1,1) = 30;
        ReqTheta2(1,1) = 30;
        ReqTheta1(2,1) = 0;
        ReqTheta2(2,1) = 0;
        ReqTheta1(3,1) = 0;
        ReqTheta2(3,1) = 0;
        if(childGenome(arm,1)==1)
            weightsTheta1(1,1) = -(bi2de(childGenome(arm,2:12)));
        else
           weightsTheta1(1,1) = (bi2de(childGenome(arm,2:12)));
        end
        if(childGenome(arm,13)==1)
            weightsTheta1(2,1) = -(bi2de(childGenome(arm,14:24)));
        else
            weightsTheta1(2,1) = (bi2de(childGenome(arm,14:24)));
        end
        if(childGenome(arm,25)==1)
            weightsTheta2(1,1) = -(bi2de(childGenome(arm,26:36)));
        else
            weightsTheta2(1,1) = (bi2de(childGenome(arm,26:36)));
        end
        if(childGenome(arm,37)==1)
            weightsTheta2(2,1) = -(bi2de(childGenome(arm,38:48)));
        else
            weightsTheta2(2,1) = (bi2de(childGenome(arm,38:48)));
        end
        collision = 0;
        holdingTarget = 0;
        generationInitialWeights(arm,:)=[weightsTheta1(1,1) weightsTheta1(2,1) weightsTheta2(1,1) weightsTheta2(2,1)];
        %================================FIRST 2.5ms=======================
        Theta1Pred(2,1) = mod(theta1(1,1)*weightsTheta1(1,1),360);
        Theta2Pred(2,1) = mod(theta2(1,1)*weightsTheta2(1,1),360);
        if((theta1(1,1)<=mod(Theta1Pred(2,1),360))&&(theta2(1,1)<=mod(Theta2Pred(2,1),360)))
          for angle1 = theta1(1,1):mod(Theta1Pred(2,1),360)
              for angle2 = theta2(1,1):mod(Theta2Pred(2,1),360)
                  X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X = round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                  if(((round(Y,3)== Link1RHS)&& X>=0 && Y>=0 && X<X1 && Y<Y1)|| ((round(Y,3)==Link2RHS)&& X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                  end
                  end
              end
              if(collision ==1)
              break;
              end
          end
       elseif((theta1(1,1)<=mod(Theta1Pred(2,1),360))&&(theta2(1,1)>mod(Theta2Pred(2,1),360)))
           for angle1 = theta1(1,1):mod(Theta1Pred(2,1),360)
              for angle2 = mod(Theta2Pred(2,1),360):theta2(1,1)
                  X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X = round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                  if(((round(Y,3)== Link1RHS)&& X>=0 && Y>=0 && X<X1 && Y<Y1)|| ((round(Y,3)==Link2RHS)&& X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                  end
                  end
              end
              if(collision == 1)
                  break;
              end
           end
          elseif((theta1(1,1)>mod(Theta1Pred(2,1),360))&&(theta2(1,1)<=mod(Theta2Pred(2,1),360)))
           for angle1 = mod(Theta1Pred(2,1),360):theta1(1,1)
              for angle2 = theta2(1,1):mod(Theta2Pred(2,1),360)
                 X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X = round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                  if(((round(Y,3)== Link1RHS)&& X>=0 && Y>=0 && X<X1 && Y<Y1)|| ((round(Y,3)==Link2RHS)&& X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                  end
                  end
              end
              if(collision == 1)
                  break;
              end
           end
          elseif((theta1(1,1)>mod(Theta1Pred(2,1),360))&&(theta2(1,1)>mod(Theta2Pred(2,1),360)))
           for angle1 = mod(Theta1Pred(2,1),360):theta1(1,1)
              for angle2 = mod(Theta2Pred(2,1),360):theta2(1,1)
                 X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X = round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                  if(((round(Y,3)== Link1RHS)&& X>=0 && Y>=0 && X<X1 && Y<Y1)|| ((round(Y,3)==Link2RHS)&& X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <1)   
                      collision = collision + 1;
                      ReqTheta1(2,1) = angle1;
                      ReqTheta2(2,1) = angle2;
                      break;
                      end
                  end
                  end
              end
              if(collision == 1)
                  break;
              end
          end
        end
       errorTheta1(1,1) = mod(Theta1Pred(2,1),360) - ReqTheta1(2,1);
       errorTheta2(1,1) = mod(Theta2Pred(2,1),360) - ReqTheta2(2,1);
       prevWeights1(1,1) = weightsTheta1(1,1);
       prevWeights2(1,1) = weightsTheta2(1,1);
       weightsTheta1(1,1) = weightsTheta1(1,1)-(0.001*errorTheta1(1,1));
       weightsTheta2(1,1) = weightsTheta2(1,1)-(0.001*errorTheta2(1,1));
       %===========================NEXT 2.5ms==============================
       Theta1Pred(3,1) = mod(Theta1Pred(2,1)*weightsTheta1(2,1),360);
       Theta2Pred(3,1) = mod(Theta2Pred(2,1)*weightsTheta2(2,1),360);
       if((mod(Theta1Pred(2,1),360)<=mod(Theta1Pred(3,1),360))&&(mod(Theta2Pred(2,1),360)<=mod(Theta2Pred(3,1),360)))
          for angle1 = mod(Theta1Pred(2,1),360):mod(Theta1Pred(3,1),360)
              for angle2 = mod(Theta2Pred(2,1),360):mod(Theta2Pred(3,1),360)
                  X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X =  round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                 if((round(Y,3)== Link1RHS && X>=0 && Y>=0 && X<X1 && Y<Y1) || (round(Y,3)==Link2RHS && X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                 end
                  end
              end
              if(collision == 2)
                  break;
              end
          end
       elseif((mod(Theta1Pred(2,1),360)<=mod(Theta1Pred(3,1),360))&&(mod(Theta2Pred(2,1),360)>=mod(Theta2Pred(3,1),360)))
           for angle1 = mod(Theta1Pred(2,1),360):mod(Theta1Pred(3,1),360)
              for angle2 = mod(Theta2Pred(3,1),360):mod(Theta2Pred(2,1),360)
                 X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X =  round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                 if((round(Y,3)== Link1RHS && X>=0 && Y>=0 && X<X1 && Y<Y1) || (round(Y,3)==Link2RHS && X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                 end
                  end
              end
              if(collision == 2)
                  break;
              end
           end
          elseif((mod(Theta1Pred(2,1),360)>=mod(Theta1Pred(3,1),360))&&(mod(Theta2Pred(2,1),360)<=mod(Theta2Pred(3,1),360)))
           for angle1 = mod(Theta1Pred(3,1),360):mod(Theta1Pred(2,1),360)
              for angle2 = mod(Theta2Pred(2,1),360):mod(Theta2Pred(3,1),360)
                X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X =  round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                 if((round(Y,3)== Link1RHS && X>=0 && Y>=0 && X<X1 && Y<Y1) || (round(Y,3)==Link2RHS && X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                 end
                  end
              end
              if(collision == 2)
                  break;
              end
           end
          elseif((mod(Theta1Pred(2,1),360)>=mod(Theta1Pred(3,1),360))&&(mod(Theta2Pred(2,1),360)>=mod(Theta2Pred(3,1),360)))
           for angle1 = mod(Theta1Pred(3,1),360):mod(Theta1Pred(2,1),360)
              for angle2 = mod(Theta2Pred(3,1),360):mod(Theta2Pred(2,1),360)
                X1 = L*cosd(angle1);
                  Y1 = L*sind(angle1);
                  X2 = L*(cosd(angle1)+cosd(angle1+angle2));
                  Y2 = L*(sind(angle1)+sind(angle1+angle2));
                  if(angle1+angle2 == 90)
                      Link1X =  round(L*cos(angle1*pi/180),3);
                      if(round(X,3) == Link1X)
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                      end
                  else
                      Link1RHS = round(tan(angle1*pi/180)*X,3);
                      Link2RHS = round((tan((angle1+angle2)*pi/180)*X)+(L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)-(tan((angle1+angle2)*pi/180)*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180))))),3);
                 if((round(Y,3)== Link1RHS && X>=0 && Y>=0 && X<X1 && Y<Y1) || (round(Y,3)==Link2RHS && X>=X1 && Y>=Y1 && X<=X2 && Y<=Y2))
                      if(collision <2)   
                      collision = collision + 1;
                      ReqTheta1(3,1) = angle1;
                      ReqTheta2(3,1) = angle2;
                      break;
                      end
                 end
                  end
              end
              if(collision == 2)
                  break;
              end
          end
       end
       errorTheta1(2,1) = mod(Theta1Pred(3,1),360) - ReqTheta1(3,1);
       errorTheta2(2,1) = mod(Theta2Pred(3,1),360) - ReqTheta2(3,1);
       errorArm(arm,:) = [errorTheta1(2,1) errorTheta2(2,1)]; 
       prevWeights1(2,1) = weightsTheta1(2,1);
       prevWeights2(2,1) = weightsTheta2(2,1);
       weightsTheta1(2,1) = weightsTheta1(2,1)-(0.001*errorTheta1(2,1));
       weightsTheta2(2,1) = weightsTheta2(2,1)-(0.001*errorTheta2(2,1));
       generationFinalWeights(arm,:)=[weightsTheta1(1,1) weightsTheta1(2,1) weightsTheta2(1,1) weightsTheta2(2,1)];
       angle1 = Theta1Pred(3,1);
       angle2 = Theta2Pred(3,1);
       calculatedX = L*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180));
       calculatedY = L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180));
       if((((X-0.1) < L*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180)))&&((X+0.1) > L*(cos(angle1*pi/180)+cos((angle1+angle2)*pi/180)))) && (((Y-0.1) < L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)))&&((Y+0.1) > L*(sin(angle1*pi/180)+sin((angle1+angle2)*pi/180)))))
           holdingTarget = 1;
       end
       if(weightsTheta1(1,1)<0)
           binWeightsTheta1(1,1) = 1;
       else
           binWeightsTheta1(1,1) = 0;
       end
       if(weightsTheta1(2,1)<0)
           binWeightsTheta1(13,1) = 1;
       else
           binWeightsTheta1(13,1) = 0;
       end
       if(weightsTheta2(1,1)<0)
           binWeightsTheta2(1,1) = 1;
       else
           binWeightsTheta2(1,1) = 0;
       end
       if(weightsTheta2(2,1)<0)
           binWeightsTheta2(13,1) = 1;
       else
           binWeightsTheta2(13,1) = 0;
       end
       binWeightsTheta1(2:12,1) = de2bi(abs(floor(weightsTheta1(1,1))),11);
       binWeightsTheta1(14:24,1) = de2bi(abs(floor(weightsTheta1(2,1))),11);
       binWeightsTheta2(2:12,1) = de2bi(abs(floor(weightsTheta2(1,1))),11);
       binWeightsTheta2(14:24,1) = de2bi(abs(floor(weightsTheta2(2,1))),11);
       %===========SETTING GENOME AND CALCULATING FITNESS VALUE============
       angleValues(arm,:) = [Theta1Pred' Theta2Pred'];
       armGenome(arm,:) = [binWeightsTheta1' binWeightsTheta2'];
       fitnessValue(arm,1) = (10000*holdingTarget) + collision;
       fitnessValue(arm,2) = arm;
    end
    count = 0;
    for i = 1: population
        if(fitnessValue(i,1)>200)
            count= count+1;
        end
    end
    if(count>10)
        flag = 1;
    end
       %===============SORTING WITH RESPECT TO FITNESS VALUE===============
       fitnessValue = sortrows(fitnessValue,1,'descend');
       for p = 1 : 20
          fitnessValue(p,3) = 5; 
       end
    
       %========================CREATING ROULETTE==========================
       k = 0;
       for m = 1: population
            for j = 1:fitnessValue(m,3)
                k = k+1;
                roulette(k,1) = fitnessValue(m,2);   
            end
       end
       %========================CHOOSING THE PAIRS=========================
       for l = 1 : 50
            for m = 1 : 2
            pair(l,m) = roulette(randi(k,1),1); 
            end
       end
       %========================MATING/REPRODUCTION========================
       value = 1;
       for a =1:50
            for b = 1:2
                if(b == 1)
                    childGenome(value,1:24) = armGenome(pair(a,1),1:24);
                    childGenome(value,25:48) = armGenome(pair(a,2),25:48);
                else
                    childGenome(value,1:24) = armGenome(pair(a,2),1:24);
                    childGenome(value,25:48) = armGenome(pair(a,1),25:48);
                end
            %======================INDUCING MUTATION=======================
                hit = rand;
                if(hit<=0.01)
                    changedGenome = randi(48);
                   childGenome(value,changedGenome)=~childGenome(value,changedGenome);
                end
                value = value+1;
            end
       end
    population = 100;
    generation = generation+1
    SumError(generation,:) = [0 0];
    for p = 1:population
          SumError(generation,1) = SumError(generation,1)+errorArm(p,1); 
          SumError(generation,2) = SumError(generation,2)+errorArm(p,2);
    end
    errorCompare(generation,1) = SumError(generation,1)/population;
    errorCompare(generation,2) = SumError(generation,2)/population;
    errorCompare(generation,3) = errorArm(fitnessValue(1,2),1);
    errorCompare(generation,4) = errorArm(fitnessValue(1,2),2);
    
    simAngle(generation,1) = generation;
    simAngle(generation,2) = angleValues(fitnessValue(1,2),3);
    simAngle(generation,3) = angleValues(fitnessValue(1,2),6);
end
%simulation of beginning stages
simulateData(X,Y,simAngle(1,2),simAngle(1,3),simAngle(1,1));
midWay = ceil(generation/2);
%simulation half way through the learning process
simulateData(X,Y,simAngle(midWay,2),simAngle(midWay,3),simAngle(midWay,1));
%final generation simulation
simulateData(X,Y,simAngle(generation,2),simAngle(generation,3),simAngle(generation,1));
figure();
GenerationPlot = 1:(generation);
plot(GenerationPlot,abs(errorCompare(:,1)));
hold on;
plot(GenerationPlot,abs(errorCompare(:,3)),'k');
hold off;
title('Error on theta1(most fit vs mean)');
figure();
plot(GenerationPlot,abs(errorCompare(:,2)));
hold on;
plot(GenerationPlot,abs(errorCompare(:,4)),'k');
hold off;
title('Error on theta2(most fit vs mean)');