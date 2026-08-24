def gitRun(String gitCommand, String auth = null) {
    String cmd = "git "

    if (auth) {
        cmd += "-c http.extraHeader=\"Authorization: Basic ${auth}\" "
    }

    cmd += gitCommand
    //echo "git_cmd:[${cmd}]"
    int status = sh(script: """ set +x ; ${cmd}""", returnStatus: true)

    echo "git exit status: ${status}"

    return status
}

def clone(String url, String auth = null) {
    gitRun(" clone ${url}", auth)
}

def checkout(String branch) {
    gitRun(" checkout ${branch}")
}

def createBranch(String branch) {
    gitRun(" checkout -b ${branch}")
}

def fetch(String auth = null) {
    gitRun(" fetch --all --prune", auth)
}

def pull(String auth = null) {
    gitRun(" pull", auth)
}

def rebase(String branch) {
    gitRun(" rebase ${branch}")
}

def commit(String message) {
    gitRun(" add -u && git commit -s -m \"${message}\"")
}

def commit_amend(String message) {
    gitRun(" add -u && git commit --amend -s -m \"${message}\"")
}


def push(String remote, String branch, String auth = null) {
    gitRun(" push --force-with-lease ${remote} ${branch}", auth)
}

def tag(String tag) {
    gitRun(" tag ${tag}")
}

def deleteTag(String tag) {
    gitRun(" tag -d ${tag}")
}

def status() {
    gitRun(" status")
}

def currentBranch() {
    gitRun("rev-parse --abbrev-ref HEAD")
}

def changedFiles() {
    return sh(
        script: "git diff --name-only",
        returnStdout: true
    ).trim().split("\\n")
}

def clean() {
    gitRun(" clean -fdx")
}

def resetHard() {
    gitRun(" reset --hard")
}

/**
 * Check whether a local branch exists.
 */
def branchExists(String branch) {
    return sh(
        script: "git show-ref --verify --quiet refs/heads/${branch}",
        returnStatus: true
    ) == 0
}

/**
 * Check whether a remote branch exists.
 */
def remoteBranchExists(String remote, String branch) {
    return sh(
        script: "git ls-remote --heads ${remote} ${branch}",
        returnStdout: true
    ).trim() != ""
}

/**
 * Return current commit SHA.
 */
def currentCommitSHA() {
    return sh(
        script: "git rev-parse HEAD",
        returnStdout: true
    ).trim()
}

/**
 * Print a short log.
 */
def log(int count = 10) {
    sh "git log --oneline --graph --decorate --max-count=${count}"
}

def full_log(int count = 10) {
    sh "git log -${count}"
}

/**
 * Print the configured remotes.
 */
def remotes() {
    sh "git remote -v"
}

/**
 * Run some git custom command, give argument without git.
 */
def customCmd(String command, String auth = null) {
    gitRun(" ${command}", auth)
}

def updateBranchWithRemote(Map a, String branch, String remote_branch = "main") {
    int status = sh(script: """
        set -e
        if ! git remote get-url upstream >/dev/null 2>&1; then
            git remote add upstream "https://github.com/${a.owner}/${a.manifest_repo}"
        fi
        git fetch upstream
        git fetch origin
        echo "Checking origin/${branch} ..."
        if git show-ref --verify --quiet "refs/remotes/origin/${branch}"; then
            echo "Branch origin/${branch} exists"
            if git show-ref --verify --quiet "refs/heads/${branch}"; then
                git switch "${branch}"
            else
                git switch -c "${branch}" --track "origin/${branch}"
            fi
        else
            echo "Branch origin/${branch} does not exist"
            git switch -c "${branch}" "upstream/main"
        fi

        echo "Rebasing ${branch} onto latest upstream/main"
        git rebase "upstream/${remote_branch}"
        echo "Rebase completed successfully"
        #git push --force-with-lease origin "${branch}"
    """, returnStatus: true)

    return status == 0
}


return this
