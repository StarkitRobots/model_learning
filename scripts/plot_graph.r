
args <- commandArgs(TRUE)

if (length(args) < 1) {
    print("Usage: ... <logFiles>");
    quit(status=1)
}
