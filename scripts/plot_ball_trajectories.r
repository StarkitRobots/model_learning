library(ggplot2)
library(optparse)

# Plot sequences with different types of data (ignored, input, predicted, measured)
debugPlot <- function(data_path, output_prefix = "seq_")
{
    data <- read.csv(data_path)
    for (seq in unique(data$seq)) {
        seqData <- data[which(data$seq == seq),]
        seqData$time <- seqData$time - min(seqData$time)
        g <- ggplot(seqData, aes(x=ball_x,y=ball_y, color=data_type, group=data_type))
        g <- g + geom_point(size=0.2)
        g <- g + coord_fixed()
        ggsave(paste0(output_prefix,seq,".png"))
    }
}

# Plot all sequences on the same graph
sequencePlot <- function(data_path, output_file = "sequences.png")
{
    data <- read.csv(data_path)
    data$seq <- paste(data$log, data$traj,sep=':')
    for (seq in unique(data$seq)) {
        indices <- which(data$seq == seq)
        timeEntries <- data[indices,"time"]
        seqStart <- min(timeEntries)
        data[indices, "time"] <- timeEntries - seqStart
    }
    g <- ggplot(data, aes(x=ball_x,y=ball_y, color=time, group=seq))
    g <- g + geom_point(size=0.2)
    g <- g + facet_wrap(~seq)
    ggsave(output_file)
}

# Uses raw data from logs
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

debugPlot(input_file)
#sequencePlot(input_file, output_file)
#trajectoryPlot(input_file, mode, output_file)
