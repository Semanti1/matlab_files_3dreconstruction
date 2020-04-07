%temple 0272,0274
img1 = imread('/home/semanti/Documents/cnngeometric_pytorch/source.png');
img2 = imread('/home/semanti/Documents/cnngeometric_pytorch/target.png');
figure
imshow(img1);
title('original img1');
figure
imshow(img2)
title('original img2');
%%
% pin = [1578,1614;1578,1211;2057,1682;2021,1163]';
% pout = [1577,1732;1585,1326;2066,1807;2033,1280]';
% pin =[209,426;572,70;601,174;165,65];
% pout=[206,432;570,68;602,182;167,65];
% tform = estimateGeometricTransform(pin,pout,'projective');
tform=[0.9716,  0.0024, -0.0203;  0.0608,  0.9383, -0.0029;0,0,1];
img1_bw = imbinarize(rgb2gray(img1),0.3);
img2_bw = imbinarize(rgb2gray(img2),0.3);

xbl = 500;
xbr = 2524;
[x1,y1]=find(img1_bw==1);
[x2,y2]=find(img2_bw==1);
figure
imshow(img1_bw)
hold on;
plot(y1,x1,'rx');
title('points on image1 after segmentation');
figure
imshow(img2_bw);
hold on;
plot(y2,x2,'rx');
title('points on image2 after segmentation');
% aff = tform.T'
% inp=[x1,y1]';
%%
% aff = tform.T'
% aff = tform.T'
aff=tform
inp=[y1,x1]';
pin = inp;
figure
imshow(img1);
hold on;
plot(pin(1,:),pin(2,:),'rx');
title('points in img1')
% hold on;
% plot(1422,1486,'bd');
% x=pin;
x=inp;
q = aff * [x; ones(1, size(x,2))];
% q1 = aff * [1422;1486;1];
p = q(3,:);
y = [q(1,:)./p; q(2,:)./p];
% p1 = q1(3,:);
% y1 = [q1(1,:)./p1; q1(2,:)./p1];
% for i=1:size(y_orig,2)
%     
%     if (y2(find(x2==y_orig(1))==y_orig(1))
figure
imshow(img2);
hold on;
% plot(pout(1,:),pout(2,:),'rx');
% hold on;
plot(y(1,:),y(2,:),'go');
title('points in img2 mapped using affine')
% hold on;
% plot(y1(1,:),y1(2,:),'bd');
%%
k1=[1520.400000 0.000000 302.320000 ;0.000000 1525.900000 246.870000; 0.000000 0.000000 1.000000];
Rt1 = [-0.29387949152391557000 -0.92616116390293235000 -0.23634792772438429000;...
0.68238554553231978000 -0.03014326033297054500 -0.73037069430875790000;...
0.66931667520703775000 -0.37592137786306423000 0.64085747710027674000];
t1 = [0.06225356349589199200 0.00226339758172542340 0.59810147012263748000];
    
cam1 = k1*[Rt1,t1'];
k2 =[1520.400000 0.000000 302.320000; 0.000000 1525.900000 246.870000; 0.000000 0.000000 1.000000];
Rt2 = [-0.20595429120762707000 -0.92987223499940797000 -0.30482856905213124000;...
0.86067467098703787000 -0.02390185791323693600 -0.50859395583279310000;...
0.46564142927205016000 -0.36710533606241880000 0.80524017012316307000];
t2 = [0.06333213439936090400 0.00339215431226642420 0.59558612978872316000];
cam2 = k2*[Rt2,t2'];
world = triangulate(x',y',cam1',cam2');
scaled = world.*10000;
figure
plot(scaled(:,1),scaled(:,2))
ptCld = pointCloud(scaled);
figure
pcshow(ptCld)
title('ptcld')