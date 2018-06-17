library(ggplot2)
library(optparse)

trajectoryPlot <- function(data_path, mode, output_file = "analysis.png")
{
    xData <- "ySelf"
    yData <- "xSelf"
    if (mode == "speed") {
        xData <- "vySelf"
        yData <- "vxSelf"
    }
    data <- read.csv(data_path)
    for (log in unique(data$log)) {
        indices <- which(data$log == log)
        timeEntries <- data[indices,"time"]
        logStart <- min(timeEntries)
        data[indices, "time"] <- timeEntries - logStart
    }
    g <- ggplot(data,
                aes_string(x=xData, y=yData,
                           group="log",
                           color="time"))
    g <- g + geom_point(size=0.2)
    g <- g + facet_wrap(~log)
    g <- g + theme_bw()
#    g <- g + coord_cartesian(ylim=c(-15,-5))
    ggsave(output_file)
}

# Script OPTIONS
option_list <- list(
    make_option(c("-m","--mode"), type="character", default="pos",
                help="Choose graph data [pos|speed, default: %default]"),
    make_option(c("-o","--output"), type="character", default="graph.png",
                help="Path to write image")

)
parser <- OptionParser(usage="%prog [options] <file>",
                       option_list = option_list)

args <- commandArgs(TRUE)

# Read Options
cmd <- parse_args(parser, args, positional_arguments=1)

mode <- cmd$options$mode
input_file <- cmd$args[1]
output_file <-cmd$options$output

trajectoryPlot(input_file, mode, output_file)