syms j0x j0y j0z j1x j1y j1z i0x i0y i0z i1x i1y i1z
Xj_true = [1,0,0,j0x; 0 1 0 j0y; 0 0 1 j0z; 0 0 0 1];
Xj = [1,0,0,j1x; 0 1 0 j1y; 0 0 1 j1z; 0 0 0 1];
Xi_true = [1,0,0,i0x; 0 1 0 i0y; 0 0 1 i0z; 0 0 0 1];
Xi = [1,0,0,i1x; 0 1 0 i1y; 0 0 1 i1z; 0 0 0 1];
Xj*inv(Xj_true)*Xi_true*inv(Xi)