function simulateData(Xtarget,Ytarget,angleLink1,angleLink2,generation)
    robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',3);
    L1 = 5;
    L2 = 5;
    body = robotics.RigidBody('link1');
    joint = robotics.Joint('joint1', 'revolute');
    setFixedTransform(joint,trvec2tform([0 0 0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'base');
    body = robotics.RigidBody('link2');
    joint = robotics.Joint('joint2','revolute');
    setFixedTransform(joint, trvec2tform([L1,0,0]));
    joint.JointAxis = [0 0 1];
    body.Joint = joint;
    addBody(robot, body, 'link1');
    body = robotics.RigidBody('tool');
    joint = robotics.Joint('fix1','fixed');
    setFixedTransform(joint, trvec2tform([L2, 0, 0]));
    body.Joint = joint;
    addBody(robot, body, 'link2');
    showdetails(robot)
    k =1;
    if(angleLink1>30)
        for i = 30:angleLink1
            qs(k,1) = i*pi/180;
            qs(k,2) = 30*pi/180;
            k = k+1;
        end
    else
        for i = 30:-1:angleLink1
            qs(k,1) = i*pi/180;
            qs(k,2) = 30*pi/180;
            k = k+1;
        end
    end 
    if(angleLink2>30)
        for j = 30:angleLink2
            qs(k,2) = (j)*pi/180;
            qs(k,1) = i*pi/180;
            k = k+1;
        end
    else
        for j = 30:-1:angleLink2
            qs(k,2) = (j)*pi/180;
            qs(k,1) = i*pi/180;
            k = k+1;
        end
    end
    figure
    show(robot,qs(1,:)');
    view(2)
    ax = gca;
    ax.Projection = 'orthographic';
    hold on
    for l = 1:size(qs,1)
        points(l,1) = Xtarget;
        points(l,2) = Ytarget;
    end
    scatter(points(:,1),points(:,2),'k');
    title(generation);
    axis([-11 11 -11 11])
    framesPerSecond = 15;
    r = robotics.Rate(framesPerSecond);
    for i = 1:size(qs,1)
        show(robot,qs(i,:)','PreservePlot',false);
        drawnow
        waitfor(r);
    end
end