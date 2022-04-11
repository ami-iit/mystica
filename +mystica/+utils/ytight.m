function ytight
  %
  % function ytight
  %
  % ytight
  %
  % Set axis tight only on y-axes

  xl=xlim; % retrieve auto x-limits
  axis tight   % set tight range
  xlim(xl)  % restore x limits
end
