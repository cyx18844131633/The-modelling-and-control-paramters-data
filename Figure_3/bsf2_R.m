function [ f2 ] = bsf2_R( bsStates,bsParams )
   f2 = -bsStates.Ia_0\bsStates.wIw_A;
end