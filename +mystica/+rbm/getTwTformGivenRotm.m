function twTform_PrA_PrB = getTwTformGivenRotm(rotm_a_b)

    % twistVel_A_B_P = twTform_PrA_PrB * twistVel_B_B_P
    % V^ {A}_ {B,P} = T^ {P[A]}_ {P[B]} V^ {B}_ {B,P}

    twTform_PrA_PrB = [rotm_a_b zeros(3); zeros(3) rotm_a_b];

end
