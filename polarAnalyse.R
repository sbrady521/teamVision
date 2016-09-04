polartable <- read.table("polardata", header = FALSE)

#print(polartable)

png(filename = "polarplot.png", width = 1000, height = 500)

smoothScatter(polartable[,2], polartable[,1], 
        main = "Hough Transformation Data", xlab = "Theta", ylab = "r")

dev.off()
