    
import matplotlib.pyplot as plt
import numpy as np

def graph(inputs , ylabel ):   
    labels = ['Astar', 'Dijkstra', 'PRM', 'Voronoi']
    average_means = [round(x / 70,2) for x in inputs]
    total_means = [round(x ,2) for x in inputs]

    x = np.arange(len(labels))  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width/2, average_means, width, label='Average')
    rects2 = ax.bar(x + width/2, total_means, width, label='Total')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel(ylabel)
    ax.set_title('Total ' + ylabel +' by Algorithm')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()


    def autolabel(rects):
        """Attach a text label above each bar in *rects*, displaying its height."""
        for rect in rects:
            height = rect.get_height()
            ax.annotate('{}'.format(height),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')


    autolabel(rects1)
    autolabel(rects2)

    fig.tight_layout()

    plt.show()
    return