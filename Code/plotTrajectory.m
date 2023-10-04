function plotTrajectory(trajectory)
t = 0:1:269;
xt(1,t) = trajectory(1,t);
yt(1,t) = trajectory(2,t);
zt(1,t) = trajectory(3,t);
plot3(xt,yt,zt,'-o','MarkerIndices',200)
end

