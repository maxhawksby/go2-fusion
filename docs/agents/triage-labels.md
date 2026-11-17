# Triage labels

These five labels say what an open issue needs next. An issue has at most one of them.

| Label | What it means | Who moves it out |
| --- | --- | --- |
| `needs-triage` | New. Nobody has decided what to do with it yet. | Max reads it and swaps in one of the other four labels. |
| `needs-info` | We can't act yet. We are waiting for the person who opened it to answer a question. | That person answers in a comment. Then Max triages it again. |
| `ready-for-agent` | Fully written down. An AI agent can do it alone, with no questions. | Adding the label starts Claude on GitHub, which opens a pull request on a branch named for whoever added it and never merges. A run that ends with no pull request and no comment shows red. An issue can name its starting branch on a line that starts `Base branch:`. The issue closes when that pull request merges. |
| `ready-for-human` | Ready, but a person has to do it, for example on the robot or with a judgment call. | A teammate assigns themselves. The issue closes when the work is done. |
| `wontfix` | We decided not to do it. | Nobody. Max closes the issue with a comment that says why. |
