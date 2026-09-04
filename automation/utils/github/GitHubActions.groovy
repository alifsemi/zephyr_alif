import groovy.json.JsonOutput
import groovy.json.JsonSlurper

private def request(String method, String path, Object body = null, String token) {
    String github_api = "https://api.github.com"
    String json       = body ? JsonOutput.toJson(body).replace("'", "'\\''") : ""
    String data       = body ? "-d ${json}" : ""
    String cmd        = ""

    cmd += "curl -sS "
    cmd += "-X ${method} "
    cmd += "-H 'Authorization: Bearer ${token}' "
    cmd += "-H 'Accept: application/vnd.github+json' "
    cmd += "-H 'X-GitHub-Api-Version: 2022-11-28' "

    if (body != null) {
        cmd += "${data} "
    }

    cmd += ' "' + github_api + path + '"'

    //echo "GitHub API: ${method} ${path} ::${cmd}"
    //def response = sh(script: cmd, returnStdout: true).trim()
    def response = sh(script: """ ${cmd} -w "\\nHTTP_STATUS:%{http_code}" """,
                        returnStdout: true).trim()
    if (!response) {
        echo "Empty response from GitHub"
        return [:]
    }

    def lines    = response.readLines()
    def status   = lines[-1].replace("HTTP_STATUS:", "").trim()
    def respBody = lines.size() > 1 ? lines[0..-2].join("\n") : ""

    echo "HTTP Status: ${status}"
    //echo "Response: ${respBody}"

    if (!(status in ["200", "201", "204"])) {
        echo "WARNING: GitHub request API failed. HTTP ${status}"
        echo "Response: ${respBody}"
    }

    if (!respBody) {
        return [:]
    }

    try {
        return new JsonSlurper().parseText(respBody)
    } catch (Exception e) {
        echo "Response is not JSON"
        return respBody
    }
}

def createPullRequest(Map a) {
    String body= a.body ?: ""
    String argJson = '{' +
        '"title": ' + '"' + a.title + '", ' +
        '"head" : ' + '"' + a.head  + '", ' +
        '"base" : ' + '"' + a.base  + '", ' +
        '"body" : ' + '"' + a.body  + '"'   +
        '}'

    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/pulls",
        argJson,
        a.token
    )
}

def getPullRequest(Map a) {
    request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.changeId}",
        null,
        a.token
    )
}

def listPullRequests(Map a) {
    def prs = []
    def git_api_path ="/repos/${a.owner}/${a.repo}/pulls?state=${a.state ?: 'open'}&per_page=100"

    def result = request(
        "GET",
        git_api_path,
        null,
        a.token
    )

    if (!(result instanceof List)) {
        return []
    }

    for (def pr : result) {
        prs << [
            changeId: pr.number,
            body  : pr.body?.toString(),
            title : pr.title?.toString(),
            state : pr.state?.toString()
        ]
    }

    return prs
}

def updatePullRequest(Map a) {
    request(
        "PATCH",
        "/repos/${a.owner}/${a.repo}/pulls/${a.changeId}",
        a.body,
        a.token
    )
}

def closePullRequest(Map a) {
    request(
        "PATCH",
        "/repos/${a.owner}/${a.repo}/pulls/${a.changeId}",
        [state: "closed"],
        a.token
    )
}

def getPullRequestTitle(Map a) {
    def result = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.changeId}",
        null,
        a.token
    )

    if (result instanceof Map && result.title != null) {
        return result.title.toString()
    }

    return ""
}

def getPullRequestBody(Map a) {
    def result = request(
        "GET",
        "/repos/${a.owner}/${a.repo}/pulls/${a.changeId}",
        null,
        a.token
    )

    if (result instanceof Map && result.body != null) {
        return result.body.toString()
    }

    return ""
}

def addLabel(Map a) {
    String argJson = '[ "' + a.label + '" ]'
    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/issues/${a.changeId}/labels",
        argJson,
        a.token
    )
}

def removeLabel(Map a) {
    request(
        "DELETE",
        "/repos/${a.owner}/${a.repo}/issues/${a.changeId}/labels/${a.label}",
        null,
        a.token
    )
}

def listLabels(Map a) {
    request(
        "GET",
        "/repos/${a.owner}/${a.repo}/issues/${a.changeId}/labels",
        null,
        a.token
    )
}

def addComment(Map a) {
    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/issues/${a.changeId}/comments",
        [body: a.comment],
        a.token
    )
}

def updateComment(Map a) {
    request(
        "PATCH",
        "/repos/${a.owner}/${a.repo}/issues/comments/${a.commentId}",
        [body: a.comment],
        a.token
    )
}

def deleteComment(Map a) {
    request(
        "DELETE",
        "/repos/${a.owner}/${a.repo}/issues/comments/${a.commentId}",
        null,
        a.token
    )
}

def requestChanges(Map a) {
    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/pulls/${a.changeId}/reviews",
        [
            event: "REQUEST_CHANGES",
            body : a.body
        ],
        a.token
    )
}

def getRef(Map a) {
    request(
        "GET",
        "/repos/${a.owner}/${a.repo}/git/ref/heads/${a.branch}",
        null,
        a.token
    )
}

def createBranch(Map a) {
    def ref = getRef(
        owner: a.owner,
        repo: a.repo,
        branch: a.from,
        token: a.token
    )

    request(
        "POST",
        "/repos/${a.owner}/${a.repo}/git/refs",
        [
            ref: "refs/heads/${a.branch}",
            sha: ref.object.sha
        ],
        a.token
    )
}

def deleteBranch(Map a) {
    request(
        "DELETE",
        "/repos/${a.owner}/${a.repo}/git/refs/heads/${a.branch}",
        null,
        a.token
    )
}

def branchExists(Map a) {
    try {
        getRef(a)
        return true
    } catch (Exception ignored) {
        return false
    }
}

return this
