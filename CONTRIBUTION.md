# Team 5144 FRC 2026 Code Contribution Guide

This guide outlines the contribution process for working with the team repository. Following this guide ensures our codebase remains well organized, maintained, and managed. 

---

## Team Repository 
**Organization**  | LISD-TECHnicians |  
**Repository Name** | 2026-REBUILT |  
**Master Branch** | main |  

---

## General Contribution Rules
1. **Do not commit to the "main" branch directly:** Checkout a new branch for any new features. Merge to main can occur post pull request.
2. **Pull before push:** Pull to ensure local environment is current before attempting a push.
3. **Code must Compile:** No broken or partially complete code. 
4. **Coherent and relevant commits:** If adding a commit message please ensure it relates to what you did.
5. **Review and reflect:** Please review all work and contribution steps before attempting a push. 

--- 
 
## Contribution Steps
1. **Syncing** 
* Cycle local workspace to "main" branch with 'checkout' command.
* Get latest version of the "main" branch with the 'pull' command.
``` bash
    git checkout main
    git pull origin main
```
2. **Create Branch**
* Create a new branch for the feature you are working on with 'checkout' command using the '-b' flag. Ensure name is logical. 
``` bash
    git checkout -b your-branch-name-here
```
3. **Build and Verify**
* Build your source code out. Verify it runs with the robot WPILib 'Build' command.
4. **Add and Commit**
* Add the file(s) you wish to track with git using the 'add' command. 
* Use commit to save progress and to add a clear message for the work completed. 
```bash
   git add your-files-to-track-here
   git commit -m "description of your work done here"
```
5. **Push Branch**
* Push your branch to the origin in question with the 'push' command.
```bash
   git push origin your-branch-name-here
```
6. **Create PR (Pull Request)**
* Open a pull request in GitHub for the repo to have changes approved and merged into "main" branch. Mentors will be approving or denying all PRs.

## Git Pro Handbook
Handy manual to reference if not sure about a feature of Git. **Super useful resource!**
* [ProGit Handbook](https://git-scm.com/book/en/v2)

## GitHub Help / Learning 
See this webpage if you need some additional assistance learning the GitHub landscape.
* [GitHub Learning Resource Page](https://docs.github.com/en/get-started/start-your-journey/git-and-github-learning-resources)