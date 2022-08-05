function x = skewVee(X)
    % function x = skewVee(X)

    X_skew = (X - X.')/2;

    % X = [ 0       X(1,2)  X(1,3);
    %      -X(1,2)  0       X(2,3);
    %      -X(1,3)  X(2,3)  0  ];

    % X = [ 0      -x(3)    x(2);
    %       x(3)    0      -x(1);
    %      -x(2)    x(1)    0  ];

    x = [-X_skew(2,3);X_skew(1,3);A-X_skew(1,2)];

end
