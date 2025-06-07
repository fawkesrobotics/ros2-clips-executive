<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.8">
  <compound kind="class">
    <name>cx::ClipsPlugin</name>
    <filename>classcx_1_1ClipsPlugin.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>initialize</name>
      <anchorfile>classcx_1_1ClipsPlugin.html</anchorfile>
      <anchor>a6bdb6ad84acf811daf6facf8a5497e2f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>finalize</name>
      <anchorfile>classcx_1_1ClipsPlugin.html</anchorfile>
      <anchor>a3aa0e046356cc938b853e414c539cb60</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>clips_env_init</name>
      <anchorfile>classcx_1_1ClipsPlugin.html</anchorfile>
      <anchor>ae76613ee6d5866e5ffb7b8ac14aaaa03</anchor>
      <arglist>(LockSharedPtr&lt; clips::Environment &gt; &amp;env)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>clips_env_destroyed</name>
      <anchorfile>classcx_1_1ClipsPlugin.html</anchorfile>
      <anchor>acd6b69af115c20027831a5014a43cbe0</anchor>
      <arglist>(LockSharedPtr&lt; clips::Environment &gt; &amp;env)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::string</type>
      <name>plugin_name_</name>
      <anchorfile>classcx_1_1ClipsPlugin.html</anchorfile>
      <anchor>aace7c9a1941b76a19a23e6881ec26906</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>rclcpp_lifecycle::LifecycleNode::WeakPtr</type>
      <name>parent_</name>
      <anchorfile>classcx_1_1ClipsPlugin.html</anchorfile>
      <anchor>a5c9c64f5ab925a688f022554d3985342</anchor>
      <arglist></arglist>
    </member>
  </compound>
</tagfile>
