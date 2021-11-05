%% Plot a horizontal or vertical line at coordinate Y
% Y = vector of heights to plot
% varargin: if [] -> plot horizontal
%           if nonempty -> plot vertical

function [ ] = plotLine(Y,varargin)
    
    if nargin==1
        line = 'horizontal';
    else
        line = varargin{1};
    end
    ax = gca;
    if strcmp(line,'horizontal')
        xlim = ax.XLim;
        hold on
        for i=1:length(Y)
            plot(xlim,[Y(i) Y(i)],'--k')
        end
    else
        ylim = ax.YLim;
        hold on
        for i=1:length(Y)
            plot([Y(i) Y(i)],ylim,'--k')
        end
    end
end