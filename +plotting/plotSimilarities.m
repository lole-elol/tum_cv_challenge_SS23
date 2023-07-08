function plotSimilarities(similarity)
% PLOTSIMILARITIES Plots the similarity matrix

figure;
imagesc(similarity);
colorbar;
title('Similarity matrix');
xlabel('Image index');
ylabel('Image index');

% show similarity values
for i = 1:size(similarity, 1)
    for j = 1:size(similarity, 2)
        text(j, i, sprintf('%.2f', similarity(i, j)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
end

end