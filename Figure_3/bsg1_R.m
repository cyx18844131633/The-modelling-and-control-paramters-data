function [ g1 ] = bsg1_R( bsStates,bsParams )
   g1 = amDyEulerJaco( bsStates.x1,'w2e');
end