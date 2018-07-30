function [ g2 ] = bsg2_R( bsStates,bsParams )
   g2 = bsStates.Ia_0\eye(3);
end