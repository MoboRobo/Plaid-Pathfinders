classdef lineMapLocalizer_Ben < handle
 %mapLocalizer A class to match a range scan against a map in
 % order to find the true location of the range scan relative to
 % the map.

 properties(Constant)
    maxErr = 0.05; % 5 cm
    minPts = 5; % min # of points that must match
 end

 properties(Access = private)
 end

 properties(Access = public)
    lines_p1 = [];
    lines_p2 = [];
    gain = 0.3;
    errThresh = 0.01;
    gradThresh = 0.0005;
 end
 
function obj =
    lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
    % create a lineMapLocalizer
    obj.lines_p1 = lines_p1;
    obj.lines_p2 = lines_p2;
    obj.gain = gain;
    obj.errThresh = errThresh;
    obj.gradThresh = gradThresh;
end

function ro2 = closestSquaredDistanceToLines(obj,pi)
    % Find the squared shortest distance from pi to any line
    % segment in the supplied list of line segments.
    % pi is an array of 2d points
    % throw away homogenous flag
    pi = pi(1:2,:);
    r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
    for i = 1:size(obj.lines_p1,2)
        [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
        obj.lines_p1(:,i),obj.lines_p2(:,i));
    end
    ro2 = min(r2Array,[],1);
end

function ids = throwOutliers(obj,pose,ptsInModelFrame)
    % Find ids of outliers in a scan.
    worldPts = pose.bToA()*ptsInModelFrame;
    r2 = obj.closestSquaredDistanceToLines(worldPts);
    ids = find(r2 > maxErr*maxErr);
end

function avgErr2 = fitError(obj,pose,ptsInModelFrame)
    % Find the variance of perpendicular distances of
    % all points to all lines
    
     % transform the points
    worldPts = pose.bToA()*ptsInModelFrame;

    r2 = obj.closestSquaredDistanceToLines(worldPts);
    r2(r2 == Inf) = [];
    err2 = sum(r2);
    num = length(r2);
    if(num >= lineMapLocalizer.minPts)
        avgErr2 = err2/num;
    else
    % not enough points to make a guess
    avgErr2 = inf;
    end
end

function [err2_Plus0,J] = getJacobian(obj,poseIn,modelPts)
    % Computes the gradient of the error function
    currErr = fitError(obj,poseIn,modelPts);
    
    eps = 1e-9;
    dx = [eps ; 0.0 ; 0.0];
    dxPoseErr = fitError(obj,pose(poseIn.getPose+dx),modelPts);
    dE_dx = (1/eps)*(dxPoseErr-currErr);
    
    dy = [0.0; eps; 0.0];
    dyPoseErr = fitError(obj,pose(poseIn.getPose+dy),modelPts);
    dE_dy = (1/eps)*(dyPoseErr-currErr);
    
    dTheta = [0.0; 0.0; eps];
    currTheta = poseIn.getPose(3);
    newTheta = atan2(sin(currTheta + dTheta), cos(currTheta + dTheta));
    newPose = pose(poseIn.getPose(1),poseIn.getPose(2),newTheta);
    
    dThetaPoseErr = fitError(obj,newPose,modelPts);
    dE_dTheta = (1/eps)*(dThetaPoseErr-currErr);
    
    err2_Plus0 = [dE_dx, dE_dy, dE_dTheta]; 
end
 
