function [ g2 ] = bsg2_P( bsStates,bsParams )
   g2 = 1/bsStates.Ma * eye(3);
end