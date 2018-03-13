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
    ggsave(outputFile);
}


# Script OPTIONS
option_list <- list(
    make_option(c("-o","--output"), type="character", default="vision_debug.png",
                help="Output image path")
)
parser <- OptionParser(usage="%prog [options] <logFile>",
                       option_list = option_list)

args <- commandArgs(TRUE)

# Read Options
cmd <- parse_args(parser, args, positional_arguments=1)

input_file <- cmd$args[1]
output_file <- cmd$option$output

plotTagsErrors(input_file, output_file)
