# README: Using GitHub
# (This is just a modified ChatGPT response)


### **1. Cloning the Repository**

1. Open Visual Studio Code (VS Code).
2. Open the Command Palette (Press F1).
3. Select/Type `Git: Clone`.
4. Paste the repository URL: `https://github.com/your-team/reflow-oven-controller.git`. (You might be able to just select it)
5. Choose a local folder where the repo will be cloned.
6. Once cloned, open the project folder in VS Code. (This should happen automatically)

---

### **2. Basic Workflow Overview**

1. **Always pull** the latest changes from the remote repository before making any edits. (F1, then type `Git: Pull`)
2. Work on a **separate branch** (not `main`) for your changes. (Especially for new, untested code/features)
3. Commit your changes frequently with meaningful messages.
4. Push your branch to the remote repository.
5. Create a pull request to merge your changes into `main`, **do not push when working on main**.

---

### **3. Steps to Commit Changes**

1. Open the **Source Control Panel** (Ctrl+Shift+G or the Git icon in the Activity Bar) **(It's the icon below Search)**
2. Review your modified files under the "Changes" section.
3. Stage the files you want to commit by:
   - Clicking the `+` icon next to each file.
   - Alternatively, click `+` next to "Changes" to stage all files.
4. Write a meaningful commit message in the text box (e.g., "Fix LCD display issue").
5. Press **Ctrl+Enter** or click the checkmark icon (✔️) to commit the changes.

---

### **4. Pulling and Syncing Changes**

#### **Pull (Fetch + Merge)**

1. Press F1 to open the Command Pallete
2. Type `Git: Pull` and select it
   - This fetches changes from the remote repo and merges them into your current branch.
   - If someone else pushes changes, this updates your copy to the latest one 
3. Resolve any merge conflicts if they appear.
    - This happens if you make **local** changes to a line that someone else just updated by pushing, then you try pulling
    - It's fine if the lines changed are different, the changes automatically get merged and don't override your local changes

#### **Sync**

1. Syncing (via the bottom-left `Sync Changes` button) performs both a pull and a push.
   - Use this if you have changes locally and want to make sure your branch is up-to-date.
   - **Do not do this for main**, just pull from main, and **only send pull requests for main**.
   - If you make changes to main and then sync, 

---

### **5. Pushing Changes**

1. After committing your changes, click the bottom-left corner of VS Code.
2. Select `Push` from the dropdown to send your changes to the remote repository.
   - Ensure you're pushing to your own branch (not `main`).

---

### **6. Switching Branches**

1. Click the branch indicator in the bottom-left corner of VS Code.
   - It's the button that probably says "main" for you
2. Choose the branch you want to work on, or create a new one:
   - To create a new branch, click `Create New Branch from` and name it (e.g., `feature-lcd-updates`).
   - Always branch off the latest `main`.

---

### **7. Creating a Pull Request**

1. After pushing your branch, open the **GitHub website** for the repository.
2. Navigate to the **Pull Requests** tab and click `New Pull Request`.
3. Select your branch as the "compare" branch and `main` as the "base" branch.
4. Add a title and description for the pull request, explaining your changes.
5. Submit the pull request.
6. Wait for your team to review and approve it before merging into `main`.

---

### **Tips for Working with GitHub**

- **Commit Often:** Break down your changes into small, logical commits with descriptive messages.
- **Pull Before You Push:** Always pull the latest changes before pushing to avoid conflicts.
- **Communicate:** Use pull request descriptions to explain the "what" and "why" behind your changes.
