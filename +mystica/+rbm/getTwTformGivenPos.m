function twTform_Ar0_Br0 = getTwTformGivenPos(pos_0_a_b)

    % twistVel_0_A = twTform_Ar0_Br0 * twistVel_0_B
    % V^ {0}_ {0,A} = T^ {A[0]}_ {B[0]} V^ {0}_ {0,B}

    twTform_Ar0_Br0 = [eye(3) mystica.utils.skew(pos_0_a_b); zeros(3) eye(3)];
    
end
