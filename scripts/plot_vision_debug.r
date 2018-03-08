library(ggplot2)

plotTagsErrors <- function(dataPath, outputFile="vision_debug.png") {
    data <- read.csv(dataPath)
    g <- ggplot(data, aes(x=errX, y=errY, group=tagId, color=model));
    xAbsMax <- max(-min(data$errX),max(data$errX))
    yAbsMax <- max(-min(data$errY),max(data$errY))
    maxVal <- max(xAbsMax,yAbsMax)
    g <- g + geom_point(size=0.5)
    g <- g + facet_wrap(~tagId,ncol=3)
    g <- g + coord_cartesian(xlim=c(-xAbsMax,xAbsMax), ylim=c(-yAbsMax,yAbsMax))
    g <- g + theme_bw()
    ggsave(outputFile);
}



args <- commandArgs(TRUE)

if (length(args) < 1) {
    print("Usage: ... <logFiles>");
    quit(status=1)
}

plotTagsErrors(args[1])
