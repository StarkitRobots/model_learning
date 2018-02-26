library(ggplot2)

se <- function(x)
{
    sd(x) / (sqrt(length(x)))
}

ci <- function(x)
{
    1.96 * se(x)
}

analysisPlot <- function(data_path, output_file = "analysis.png")
{
    data <- read.csv(data_path)
    print(names(data))
    values <- do.call(data.frame,
                      aggregate(validationScore~reader+optimizer, data,
                                function(x) c(mean = mean(x),
                                              sd = sd(x),
                                              se = se(x),
                                              ci = ci(x))))
    plotColumn <- "validationScore"
    meanStr <- sprintf("%s.mean", plotColumn)
    ciStr <- sprintf("%s.ci", plotColumn)
    yMinStr <- sprintf("%s-%s", meanStr, ciStr)
    yMaxStr <- sprintf("%s+%s", meanStr, ciStr)
    print("initing")
    g <- ggplot(values,
                aes_string(x="reader", y=meanStr,
                           ymin = yMinStr, ymax = yMaxStr,
                           group="optimizer",
                           color="optimizer",
                           fill="optimizer"))
    print("init OK")
    g <- g + geom_ribbon(size=0,alpha = 0.3)
    g <- g + geom_point(size=2)
    g <- g + geom_line(size=0.5)
    g <- g + theme(axis.text.x  = element_text(angle=90, vjust=0.5, size=12))
    g <- g + labs(x = "nb samples by tag")
    g <- g + scale_x_log10()
    ggsave(output_file)
}

parametersPlot <- function(data_path, output_file = "analysis.png")
{
    #TODO
}

args <- commandArgs(TRUE)

if (length(args) < 1) {
    print("Usage: ... <logFiles>");
    quit(status=1)
}

analysisPlot(args[1])
