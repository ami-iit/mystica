function [twistAcc_0_a] = transformTwistAcc(pos_0_a_b,twistAcc_0_b,angVel_0_x)

    % angVel_0_x could be both angVel_0_b, angVel_0_a
    
    % twistVel_0_A = twTform_Ar0_Br0 * twistVel_0_B
    % V^ {0}_ {0,A} = T^ {A[0]}_ {B[0]} V^ {0}_ {0,B}    
    
    % twistAcc_0_A = twTform_Ar0_Br0 * twistAcc_0_B + C_0_AB
    % \dot{V}^ {0}_ {0,A} = T^ {A[0]}_ {B[0]} \dot{V}^ {0}_ {0,B} +  C^ {0}_ {AB}
        
   
    twTform_Ar0_Br0 = mystica.rbm.getTwTformGivenPos(pos_0_a_b);
    
    twistAcc_0_a = twTform_Ar0_Br0 * twistAcc_0_b - [mystica.utils.skew(angVel_0_x)^2*pos_0_a_b ; zeros(3,1)];
    
end
