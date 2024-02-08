load robustness_vector_rl.mat

t = out0.refTraj.Time;

figure(1)
subplot(3,1,1);
% a1 = axes();
plot(t, out0.drone.Data(:,10)*(180/pi), "-k", "LineWidth",1)
hold on
X = arrayfun(@(x)(x.drone.Data(:,10)),out,'UniformOutput',false);
X =cell2mat(X)*(180/pi);
X_max = max(X,[],2);
X_min = min(X,[],2);
fill([t;flip(t)],[X_min;flip(X_max)], "r", "FaceAlpha", 0.2, "EdgeColor", "None")

a2 = axes();
a2.Position = [0.500 0.6600 0.2 0.2]; % xlocation, ylocation, xsize, ysize
plot(a2, t(2001:2020), out0.drone.Data(2001:2020,10)*(180/pi), "-k", "LineWidth",1)
axis tight
hold on
X = arrayfun(@(x)(x.drone.Data(2001:2020,10)),out,'UniformOutput',false);
X =cell2mat(X)*(180/pi);
X_max = max(X,[],2);
X_min = min(X,[],2);
fill([t(2001:2020);flip(t(2001:2020))],[X_min;flip(X_max)], "r", "FaceAlpha", 0.2, "EdgeColor", "None")


subplot(3,1,2);
% a1 = axes();
plot(t, out0.drone.Data(:,11)*(180/pi), "-k", "LineWidth",1)
hold on
X = arrayfun(@(x)(x.drone.Data(:,11)),out,'UniformOutput',false);
X =cell2mat(X)*(180/pi);
X_max = max(X,[],2);
X_min = min(X,[],2);
fill([t;flip(t)],[X_min;flip(X_max)], "g", "FaceAlpha", 0.2, "EdgeColor", "None")

a2 = axes();
a2.Position = [0.500 0.6600 0.2 0.2]; % xlocation, ylocation, xsize, ysize
plot(a2, t(2001:2020), out0.drone.Data(2001:2020,11)*(180/pi), "-k", "LineWidth",1)
axis tight
hold on
X = arrayfun(@(x)(x.drone.Data(2001:2020,11)),out,'UniformOutput',false);
X =cell2mat(X)*(180/pi);
X_max = max(X,[],2);
X_min = min(X,[],2);
fill([t(2001:2020);flip(t(2001:2020))],[X_min;flip(X_max)], "g", "FaceAlpha", 0.2, "EdgeColor", "None")

subplot(3,1,3);
% a1 = axes();
plot(t, out0.drone.Data(:,12)*(180/pi), "-k", "LineWidth",1)
hold on
X = arrayfun(@(x)(x.drone.Data(:,12)),out,'UniformOutput',false);
X =cell2mat(X)*(180/pi);
X_max = max(X,[],2);
X_min = min(X,[],2);
fill([t;flip(t)],[X_min;flip(X_max)], "b", "FaceAlpha", 0.2, "EdgeColor", "None")

a2 = axes();
a2.Position = [0.500 0.6600 0.2 0.2]; % xlocation, ylocation, xsize, ysize
plot(a2, t(2001:2020), out0.drone.Data(2001:2020,12)*(180/pi), "-k", "LineWidth",1)
axis tight
hold on
X = arrayfun(@(x)(x.drone.Data(2001:2020,12)),out,'UniformOutput',false);
X =cell2mat(X)*(180/pi);
X_max = max(X,[],2);
X_min = min(X,[],2);
fill([t(2001:2020);flip(t(2001:2020))],[X_min;flip(X_max)], "b", "FaceAlpha", 0.2, "EdgeColor", "None")