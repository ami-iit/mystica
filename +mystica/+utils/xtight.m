function xtight
  %
  % function xtight
  %
  % xtight
  %
  % Set axis tight only on x-axes

  yl=ylim; % retrieve auto y-limits
  axis tight   % set tight range
  ylim(yl)  % restore y limits
end
