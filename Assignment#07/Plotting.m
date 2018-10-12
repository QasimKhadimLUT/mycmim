function [u] = Plotting(Figure, FontType, FontSize)

axes = findall(allchild(Figure), 'Type', 'Axes');
textarrows = findall(allchild(Figure), 'Type', 'TextArrow');

set(axes, 'XGrid', 'on', 'YGrid','on');
set(axes,'FontName',FontType, 'FontSize', FontSize);
set(textarrows,'FontName',FontType, 'FontSize', FontSize);
end
