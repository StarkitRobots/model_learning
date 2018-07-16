library(ggplot2)
library(optparse)

plotTagsErrors <- function(dataPath, outputFile) {
    data <- read.csv(dataPath)
    g <- ggplot(data, aes(x=errX, y=errY, group=tagId, color=model));
    xAbsMax <- max(-min(data$errX),max(data$errX))
    yAbsMax <- max(-min(data$errY),max(data$errY))
    maxVal <- max(xAbsMax,yAbsMax)
    g <- g + geom_point(size=0.5)
    g <- g + facet_wrap(~tagId,ncol=3)
    g <- g + coord_cartesian(xlim=c(-xAbsMax,xAbsMax), ylim=c(-yAbsMax,yAbsMax))
    g <- g + theme_bw()
    ggsave(outputFile,width=20,height=40);
}

plotVectorsErrors <- function(dataPath, outputFile) {
    data <- read.csv(dataPath)
    data$tagId <- as.factor(data$tagId)
    g <- ggplot(data, aes(x=obsX, y=obsY, xend=predX, yend=predY, group=model, color=tagId));
    xMin <- min(data$obsX,data$predX)
    xMax <- max(data$obsX,data$predX)
    yMin <- min(data$obsY,data$predY)
    yMax <- max(data$obsY,data$predY)
    g <- g + geom_segment(arrow=arrow(length=unit(0.1,"cm")))
    g <- g + coord_cartesian(xlim=c(xMin,xMax), ylim=c(yMin,yMax))
    g <- g + theme_bw()
    ggsave(outputFile)
}


# Script OPTIONS
option_list <- list(
    make_option(c("-o","--output"), type="character", default="vision_debug.png",
                help="Output image path"),
    make_option(c("-t","--type"), type="character", default="errors",
                help="Type of plot [errors,vectors]")
)
parser <- OptionParser(usage="%prog [options] <logFile>",
                       option_list = option_list)

args <- commandArgs(TRUE)

# Read Options
cmd <- parse_args(parser, args, positional_arguments=1)

input_file <- cmd$args[1]
output_file <- cmd$option$output

if (cmd$option$type == "errors") {
    plotTagsErrors(input_file, output_file)
} else if (cmd$option$type == "vectors") {
    plotVectorsErrors(input_file, output_file)
} else {
    print(paste0("Invalid option for type: '",cmd$option$type,"'"))
}
