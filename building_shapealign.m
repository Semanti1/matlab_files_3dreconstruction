%church 1,2
close all;
clc;
clear;

%%
img1 = imread('/home/semanti/Downloads/buildings/Tower1_p1.jpg');
img2 = imread('/home/semanti/Downloads/buildings/Tower1_p2.jpg');
% img1 = imread('/home/semanti/Documents/cnngeometric_pytorch/source.png');
% img2 = imread('/home/semanti/Documents/cnngeometric_pytorch/target.png');
figure
imshow(img1);
title('original img1');
figure
imshow(img2);
title('original img2');
%%
% pin = [1578,1614;1578,1211;2057,1682;2021,1163]';
% pout = [1577,1732;1585,1326;2066,1807;2033,1280]';
% pin =[209,426;572,70;601,174;165,65];
% pout=[206,432;570,68;602,182;167,65];
% tform = estimateGeometricTransform(pin,pout,'projective');
% tform=[  1.2486,  0.1631, -0.1511; -0.2465,  1.0720,  0.1027;0,0,1]
% tform=[ 1.0343, -0.0020,  0.0027; -0.0123,  1.0664, -0.1928;0,0,1]%cropped
tform=[ 0.7099, -0.0671,  0.4236;  0.0255,  0.9978,  0.0296;0,0,1];
img1_bw = imbinarize(rgb2gray(img1),0.3);
img2_bw = imbinarize(rgb2gray(img2),0.3);
% img1_bw=img1_bw(60:180,60:180);
% img2_bw=img2_bw(60:180,60:180);
% img1=img1(60:180,60:180);
% img2=img2(60:180,60:180);
xbl = 500;
xbr = 2524;
[l,w]=size(img1_bw);
[x1,y1]=find(img1_bw==0);
[x2,y2]=find(img2_bw==0);
% indx = find(x1>60 & x1<200);
% indy = find(y1>60 & y1<200);
% indx = find(x1>756 & x1<2268);
% indy = find(y1>756 & y1<2268);
indx = find(x1>300 & x1<2500);
indy = find(y1>300 & y1<3000);
ind = intersect(indx,indy);
x1=x1(ind);
y1=y1(ind);
figure
imshow(img1_bw);
hold on;
plot(y1,x1,'rx');
title('points on image1 after segmentation');
% indx = find(x2>30 & x2<180);
% indy = find(y2>30 & y2<180);
% indx = find(x2>756 & x2<2268);
% indy = find(y2>756 & y2<2268);
indx = find(x2>300 & x2<3000);
indy = find(y2>300 & y2<3000);
ind = intersect(indx,indy);
x2=x2(ind);
y2=y2(ind);
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
% aff=inv(tform);
aff=tform;
% aff= [1 0 10;0 1 0;0 0 1]
inp=[y1,x1]';
% inp=[x1,y1]';
pin = inp;
figure
imshow(img1);
hold on;
plot(pin(1,:),pin(2,:),'rx');
title('points in img1')
% hold on;
% plot(1422,1486,'bd');
% x=pin;
x=[x1,y1]';
% x=[y1,x1]';
q = aff * [x; ones(1, size(x,2))];
% q1 = aff * [1422;1486;1];
p = q(3,:);
y = [q(1,:)./p; q(2,:)./p];
% y(1,:)=(y(1,:)./240).*3024;
% y(2,:)=(y(2,:)./240).*4032;
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
plot(y(2,:),y(1,:),'go');
% plot(y(1,:),y(2,:),'go')
title('points in img2 mapped using affine')
% disp('k')
% hold on;
% plot(y1(1,:),y1(2,:),'bd');
%%
orig_img2_pts = [y2,x2];
aff_tr = [(y(2,:))',(y(1,:))'];
c1 = [mean(y2),mean(x2)];
c2 = [mean(y(2,:)),mean(y(1,:))];
D=c1-c2;
aff_tr_trans = [aff_tr(:,1)+D(1),aff_tr(:,2)+D(2)];
figure
imshow(img2);
hold on;
plot(y2,x2,'rx');
hold on;
plot(aff_tr_trans(:,1),aff_tr_trans(:,2),'gx');
bound_orig  = boundary(y2,x2);
bound_aff_t = boundary(aff_tr_trans(:,1),aff_tr_trans(:,2));
figure
imshow(img2);
hold on;
plot(y2(bound_orig),x2(bound_orig),'rx');
hold on;
plot(aff_tr_trans(bound_aff_t,1),aff_tr_trans(bound_aff_t,2),'gx')
% hold on;
% plot(y2,x2,'rx');
boundpts_orig = [y2(bound_orig),x2(bound_orig)];
boundpts_aff = [aff_tr_trans(bound_aff_t,1),aff_tr_trans(bound_aff_t,2)];
dist = pdist2(boundpts_orig,boundpts_aff);
[v,ind]=min(dist,[],2);
all_img1 = x';
b_img1 = [all_img1(bound_aff_t,1),all_img1(bound_aff_t,2)];
% bound_img1 = all_img1(bound_aff_t,:);
% bound_img2 = boundpts_orig(ind,:);
bound_img1 = b_img1(ind,:);
bound_img2 = boundpts_orig;
figure
imshow(img1);
hold on;
plot(bound_img1(:,2),bound_img1(:,1),'rx');
figure
imshow(img2);
hold on;
plot(bound_img2(:,1),bound_img2(:,2),'bx');
%%
    translationOfCamera2 = [70, 0, 0];
%     translationOfCamera2 = [5.6, 0, 0];
    
% % translationOfCamera2 = [0.80,0,0];
% % translationOfCamera2 = [2.5, 0, 0];
R = eye(3);% rotationOfCamera2, align with the wolrd coordinate.
T1 = [0 0 0];
T2 = translationOfCamera2;% translationOfCamera2.
extrinsicMtx1 = [R;T1];
extrinsicMtx2 = [R;T2];
% 
% opticalCenter = [500 600];
% fx = 4; fy = 4;
% sx = 1/14; sy = 1/14;
% cx = 2000; cy = 1500;
% % cx=159;cy=119;
% % cx=502;cy=424
% intrinsicMtx = [fx/sx, 0, 0; 0, fy/sy, 0; cx, cy, 1];
intrinsicMtx = [583.141567003901,0,0;0,584.782897695379,0;2150.03791355007,1287.58122795129,1];
cam1 = intrinsicMtx'*[R,T1'];
cam2 = intrinsicMtx'*[R,T2'];
% % cam1=cam1';
% % cam2=cam2';
% % k1=[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% % Rt1 = [-0.14396457836077139000 0.96965263281337499000 0.19760617153779569000;...
% %     -0.90366580603479685000 -0.04743335255026152200 -0.42560419233334673000;...
% %     -0.40331536459778505000 -0.23984130575212276000 0.88306936201487163000];
% % t1 = [-0.010415508744 -0.0294278883669 0.673097816109];
% %     
% % cam1 = k1*[Rt1,t1'];
% % k2 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% % Rt2 = [-0.23143687262851170000 0.96422332027030622000 0.12926937165558827000;...
% %     -0.65860811165749811000 -0.05749426020475298400 -0.75028640486482034000;...
% %     -0.71601208187197274000 -0.25878137856376976000 0.64835049619606955000];
% % t2 = [-0.0143392676288 -0.0315356332107 0.659314086504];
% % cam2 = k2*[Rt2,t2'];
p1=[bound_img1(:,1),bound_img1(:,2)];
p2=[bound_img2(:,2),bound_img2(:,1)];
% world = triangulate(x',y',cam1',cam2');
n1=[];
n2=[];
for i=1:5
    for j=1:length(p1)-1
        mid=(p1(j,:)+p1(j+1,:))./2;
        n1=[n1;p1(j,:)];
        n1=[n1;mid];
    end
    n1=[n1;p1(j,:)]
    p1=n1;
    for k=1:length(p2)-1
        mid2=(p2(k,:)+p2(k+1,:))/2;
        n2=[n2;p2(k,:);mid2];
    end
    n2=[n2;p2(k,:)]
    p2=n2;
end
        
    
world = triangulate(p1,p2,cam1',cam2');
% world = triangulate(p1,p2,cam1',cam2');
% ty=[y(2,:);y(1,:)]';
% tx=[x(2,:);x(1,:)]';
% world = triangulate(ty,tx,cam2',cam1');
% world = triangulate(y',x',cam2',cam1');
scaled = world.*10000;
% % scaled = world;
figure
% plot(scaled(:,1),scaled(:,2))
% ptCld1 = pointCloud(scaled);
ptCld1 = pointCloud(world);
% figure
pcshow(ptCld1)
% title('ptcld1')
% axis tight;