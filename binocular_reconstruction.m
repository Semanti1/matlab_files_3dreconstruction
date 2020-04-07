clc; clear all; close all;
% import images.
type = 'Tower'; %Church
issave = false;
num = 1;
p1 = 51;
Data = struct;
% data_path = 'C:\Users\Chenxi\Documents\MATLAB\Model_revised\Data';
% img1 = imread([data_path '\buildings\',type,num2str(num),'_p1.jpg']);
% img2 = imread([data_path '\buildings\',type,num2str(num),'_p2.jpg']);
% load([data_path '\buildings\intrinsicMat.mat']); intrinsic = ans;
img1 = imrotate(imread('/home/semanti/Downloads/buildings/Tower1_p1.jpg'),-90);
img2 = imrotate(imread('/home/semanti/Downloads/buildings/Tower1_p2.jpg'),-90);

% img1 = imrotate(img1, -90);
% img2 = imrotate(img2, -90);
% figure,
% img2 = imrotate(img2,-90);
% imshow(img2),
%%
% fpts1 = [];
% for i = length(fpts):-1:1
%     fpts1(length(fpts)+1-i,:) = fpts(i).Position;
% end
%%
%find order
file1 = ['\buildings\',type,num2str(num),'_p1.mat'];
file2 = ['\buildings\',type,num2str(num),'_p2.mat'];
Path = 'C:\Users\Chenxi\Documents\MATLAB\Model_revised\Data\';
data1 = importdata([Path,file1]);
data2 = importdata([Path,file2]);
% 
% data1_map = [];
% data2_map = [];
% 
% data1 = nestedConvexHull(data1);
% data2 = nestedConvexHull(data2);
% 
% ratio1 = data1.layer_ratio;
% ratio2 = data2.layer_ratio;
% 
% for i = 1:size(ratio1,2)
%     ref = ratio1(:,i);
%     que = ratio2(:,i);
% 
%     shift = [0:length(ref) - 1];
%     cir_ref = repmat(ref, 1, length(ref));
%     cir_que = repmat(que, 1, length(que));
%     for c = 1:length(que)
%         cir_que(:,c) = circshift(cir_que(:,c), shift(c));
%     end
% 
%     [~,shift_idx] = min(sum(abs(cir_ref - cir_que),1));
% 
%     data1_map = [data1_map; data1.layer_pt(:,i)];
%     data2_map = [data2_map; circshift(data2.layer_pt(:,i), shift(shift_idx))];
% end

% uv = zeros(length(fpts),2);
% for i = length(fpts):-1:1
%     uv(length(fpts)+1-i,:) = fpts(i).Position;
% end

uv1 = load([data_path, '\buildings\',type,num2str(num),'_p1.mat']);
uv1 = uv1.fpts;
uv2 = load([data_path, '\buildings\',type,num2str(num),'_p2.mat']);
uv2 = uv2.fpts;

% Setup world coordinate
translationOfCamera2 = [70, 0, 0];
baseLine = 70;
focalLength = 583;
xy1 = uv1;
xy2 = uv2;
cx = 2150; cy = 1287;
for i = 1:size(uv1,1)
    disparity(i) = xy2(i,1)-xy1(i,1);
    Z(i) = baseLine*focalLength/disparity(i);
    X(i) = baseLine*(xy1(i,1) + xy2(i,1)-2*cx)/(2*disparity(i));
    Y(i) = baseLine*(xy1(i,2) + xy2(i,2)-2*cy)/(2*disparity(i));
end

figure,
plot3(X, Y, Z, 'b.')
axis equal

%% Find connection

% tri = getConnection(X, Y, p1);
% tri([1,2,3,4,5,8,21,23],:) = [];
% tri = [tri; [32,33,34;34,35,36;36,37,38;38,39,40]];
% tri([1,4,5,10,11,14,16,18,19,21,22,23,26,27,28,32,43,45:48,50,51,52,55:58],:) = [];
% tri = [tri; 8,9,10; 10,11,12;12,13,14;14,15,16;16,17,18;18,19,20;20,21,22;27,28,29;29,30,31;31,32,33;33,34,35;35,36,37;37,38,39;39,40,41];

tri = delaunay(X,Y);
poly1 = polyshape(X(1:p1), Y(1:p1));
delete_list = [];
partial_list = [];
for i = 1:length(tri)
    idx = tri(i,:);
    poly2 = polyshape([X(idx(1)), X(idx(2)), X(idx(3))], [Y(idx(1)), Y(idx(2)), Y(idx(3))]);
    overlap = intersect([poly1 poly2]);

    if ~overlap.NumRegions 
        delete_list = [delete_list, i]; 
    else
        if abs(area(poly2)-area(overlap))>1e-10
            partial_list = [partial_list, i];
        end
    end
end
unconnect_idx = unique(tri(partial_list,:));
tri([delete_list,partial_list],:) = [];
% 
% test1 = polyshape(X(unconnect_idx(1:3)), Y(unconnect_idx(1:3)));
% over1 = intersect(poly1, test1);
% test2 = polyshape(X(unconnect_idx(2:4)), Y(unconnect_idx(2:4)));
% over2 = intersect(poly1, test2);
% 
% if area(over1)<1e-10
%     start1 = 2;
% else
%     if area(over2)<1e-10
%         start1 = 1;
%     end
% end
% 
% add_tri = [];
% while start1<length(unconnect_idx)
%     add_tri = [add_tri, unconnect_idx(start:start+2)];
%     start1 = start1 +2;
% end
%     
% add_tri = add_tri';
% 
% tri = [tri; add_tri];

pts = [X', Y', Z'];
faces = tri;

for i = 1:7
[pts, faces] = myLoopSubdivision2(pts, faces, 2);
end

figure,
plot3(pts(:,1), pts(:,2), pts(:,3),'.')
axis equal

affine_mtx = estimateGeometricTransform(data1,data2,'affine');
project_mtx = estimateGeometricTransform(data1,data2,'projective');

Data.Ver = pts;
Data.Tri = faces;
Data.Fpts = [X',Y',Z'];
Data.baseline = 70;
Data.img1 = img1;
Data.img2 = img2;
Data.uv1 = data1;
Data.uv2 = data2;
Data.affine_mtx = affine_mtx.T;
Data.project_mtx = project_mtx.T;

save_file = ['\buildings\',type,num2str(num),'.mat'];
% 
% if issave
%     fprintf('The .mat file is saved!')
%     save([Path,save_file], 'Data')
% end





