function quat_b_a = invQuat(quat_a_b)

    qw = quat_a_b(1); qxyz = quat_a_b(2:4);
    quat_b_a = [qw;-qxyz]/(norm(quat_a_b)^2);

end
